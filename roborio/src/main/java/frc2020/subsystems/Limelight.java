package frc2020.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;
import frc2020.util.Logger;
import frc2020.util.vision.TargetInfo;
import frc2020.util.Util;

/**
 * Subsystem for interacting with the Limelight 2
 * 
 * This class does not explicitly track the targets but rather
 * determines whether or not target data recieved is reliable.
 */
public class Limelight implements Subsystem {
    public final static int DEFAULT_PIPELINE = 0;

    public static class LimelightConfig {
        public String name = "";
        public String tableName = "";
        public double height = 0.0;
        public Transform2d turretToLens = new Transform2d();
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
    private boolean hasTarget_ = false;

    public Transform2d getTurretToLens() {
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
        io_.latency = networkTable_.getEntry("tl").getDouble(0) / 1000.0 + Constants.IMAGE_CAPTURE_LATENCY;
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
        List<TargetInfo> targets = getRawTargetInfo();
        if (hasTarget() && targets != null) {
            return targets;
        }
        return null;
    }

    /**
     * Gets the raw target info of the top two corners of the vision target.
     * This is a unit plane from -1 to 1.
     * 
     * @return Returns a list of the x and y values of the top 2 corners of 
     * the vision target with the first item being the left corner and the 
     * second being the right corner.
     */
    private synchronized List<TargetInfo> getRawTargetInfo() {
        List<double[]> corners = getTopCorners();
        if (corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) / (corners.get(1)[0] - corners.get(0)[0]);
        }

        List<TargetInfo> targets = new ArrayList<>();
        for (int i = 0; i < 2; ++i) {
            double y_pixels = corners.get(i)[0];
            double z_pixels = corners.get(i)[1];

            // Normalize the pixel coordinate to use the center of the frame
            // as (0, 0)
            // Note the negation is so that z is up and y is left to
            // correspond to the coordinate system we use on the bot
            double nY = -((y_pixels - (Constants.LIMELIGHT_RES_X / 2)) /
                (Constants.LIMELIGHT_RES_X / 2));
            double nZ = -(((Constants.LIMELIGHT_RES_Y / 2) - z_pixels) /
                (Constants.LIMELIGHT_RES_Y / 2));

            // Project the normalized coordinates onto a plane 1.0
            // unit away from the camera
            double y = Constants.VERTICAL_PLANE_WIDTH / 2 * nY;
            double z = Constants.VERTICAL_PLANE_HEIGHT / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            targets.add(target);
        }

        return targets;
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index  - top right
     */
    private List<double[]> getTopCorners() {

        // Get the list of x and y position in pixels(?) if we have a valid target
        double[] emptyArray = {};
        double[] corners = networkTable_.getEntry("tcornxy").getDoubleArray(emptyArray);
        hasTarget_ = networkTable_.getEntry("tv").getDouble(0) == 1.0;

        // If we don't have a valid target or the corners list is empty
        // something went wrong so we return null
        if (!hasTarget_ || Arrays.equals(corners, emptyArray) || corners.length <= 8 || corners.length % 2 != 0) {
            return null;
        }

        double[] xCorners = new double[corners.length/2];
        double[] yCorners = new double[corners.length/2];
        int idx = 0;
        for (int i = 1; i < corners.length; i += 2) {
            xCorners[idx] = corners[i-1];
            yCorners[idx] = corners[i];
            idx++;
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
        Logger.logDebug("Before sort: " + corners);
        corners.sort(xSort);
        Logger.logDebug("After sort: " + corners);

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
