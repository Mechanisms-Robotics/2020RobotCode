package frc2020.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;
import frc2020.util.Logger;
import frc2020.util.vision.TargetInfo;
import frc2020.util.Util;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight implements Subsystem {
    public final static int DEFAULT_PIPELINE = 0;

    public static class LimelightConfig {
        public String name = "";
        public String tableName = "";
        public double height = 0.0;
        public Pose2d turretToLens = new Pose2d();
        public Rotation2d horizontalPlaneToLens = new Rotation2d();
    }

    private NetworkTable networkTable_;

    public Limelight(LimelightConfig config) {
        config_ = config;
        networkTable_ = NetworkTableInstance.getDefault().getTable(config.tableName);
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    private LimelightConfig config_ = null;
    private PeriodicIO io_ = new PeriodicIO();
    private boolean outputsHaveChanged_ = true;
    private List<TargetInfo> targets_ = new ArrayList<>();
    private boolean hasTarget_ = false;

    public Pose2d getTurretToLens() {
        return config_.turretToLens;
    }

    public double getLensHeight() {
        return config_.height;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return config_.horizontalPlaneToLens;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        io_.latency = networkTable_.getEntry("tl").getDouble(0) / 1000.0 + Constants.IMAGE_CAPTURE_LANTECY;
        io_.givenLedMode = (int) networkTable_.getEntry("ledMode").getDouble(1.0);
        io_.givenPipeline = (int) networkTable_.getEntry("pipeline").getDouble(0);
        io_.xOffset = networkTable_.getEntry("tx").getDouble(0.0);
        io_.yOffset = networkTable_.getEntry("ty").getDouble(0.0);
        io_.area = networkTable_.getEntry("ta").getDouble(0.0);
        hasTarget_ = networkTable_.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (io_.givenLedMode != io_.ledMode || io_.givenPipeline != io_.pipeline) {
            Logger.logDebug("Table has changed from expected, retrigger!!");
            outputsHaveChanged_ = true;
        }
        if (outputsHaveChanged_) {
            networkTable_.getEntry("ledMode").setNumber(io_.ledMode);
            networkTable_.getEntry("camMode").setNumber(io_.camMode);
            networkTable_.getEntry("pipeline").setNumber(io_.pipeline);
            networkTable_.getEntry("stream").setNumber(io_.stream);
            networkTable_.getEntry("snapshot").setNumber(io_.snapshot);

            outputsHaveChanged_ = false;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(config_.name + ": Has Target", hasTarget_);
        SmartDashboard.putNumber(config_.name + ": Pipeline Latency (ms)", io_.latency);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != io_.ledMode) {
            io_.ledMode = mode.ordinal();
            outputsHaveChanged_= true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != io_.pipeline) {
            io_.pipeline = mode;

            System.out.println(io_.pipeline + ", " + mode);
            outputsHaveChanged_ = true;
        }
    }

    public synchronized void triggerOutputs() {
        outputsHaveChanged_ = true;
    }

    public synchronized int getPipeline() {
        return io_.pipeline;
    }

    public synchronized boolean hasTarget() {
        return hasTarget_;
    }

    /**
     * @return two targets that make up one hatch/port or null if less than two
     *         targets are found
     */
    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();
        if (hasTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    private synchronized List<TargetInfo> getRawTargetInfos() {
        List<double[]> corners = getTopCorners();
        if (corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) / (corners.get(1)[0] - corners.get(0)[0]);
        }

        targets_.clear();
        for (int i = 0; i < 2; ++i) {
            double y_pixels = corners.get(i)[0];
            double z_pixels = corners.get(i)[1];

            // Normilize the pixel coordinats to use the center of the frame
            // as (0, 0)
            double nY = -((y_pixels - (Constants.LIMELIGHT_RES_X / 2)) /
                ((Constants.LIMELIGHT_RES_X / 2) - 0.5));
            double nZ = -((z_pixels - (Constants.LIMELIGHT_RES_Y / 2)) /
                ((Constants.LIMELIGHT_RES_Y / 2) - 0.5));

            // Project the normilized coordinates onto a plane 1.0
            // unit away from the camera
            double y = Constants.VERTICAL_PLANE_WIDTH / 2 * nY;
            double z = Constants.VERTICAL_PLANE_HEIGHT / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            targets_.add(target);
        }

        return targets_;
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index  - top right
     */
    private List<double[]> getTopCorners() {

        // Get the list of x and y position in pixels(?) if we have a valid target
        double[] emptyArray = {};
        double[] xCorners = networkTable_.getEntry("tcornx").getDoubleArray(emptyArray);
        double[] yCorners = networkTable_.getEntry("tcorny").getDoubleArray(emptyArray);
        hasTarget_ = networkTable_.getEntry("tv").getDouble(0) == 1.0;

        // If we don't have a valid target or the corners list is empty
        // something went wrong so we return null
        if (!hasTarget_ || Arrays.equals(xCorners, emptyArray) || Arrays.equals(yCorners, emptyArray)
                || xCorners.length <= 4 || yCorners.length <= 4) {
            return null;
        }

        return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    }

    // Defines how we can can compare tranlation 2ds
    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::getX);

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {

        // Put the x and y positions of the coners into 
        // Translation2d Objcets
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        // Sort the corners by the by the x-vaules to
        // sort them from left to right
        corners.sort(xSort);

        Translation2d leftCorner = corners.get(0);
        Translation2d rightCorner = corners.get(corners.size() - 1);

        return List.of(new double[] { leftCorner.getX(), leftCorner.getY() },
                new double[] { rightCorner.getX(), rightCorner.getY() });
    }

    public double getLatency() {
        return io_.latency;
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub

    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        // TODO Auto-generated method stub

    }
}
