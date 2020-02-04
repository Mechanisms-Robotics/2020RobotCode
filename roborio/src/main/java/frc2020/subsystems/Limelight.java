package frc2020.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;
import frc2020.util.Logger;

/**
 * Subsystem for interacting with the Limelight 2
 * 
 * This class does not explicitly track the targets but rather
 * determines whether or not target data recieved is reliable.
 */
public class Limelight implements Subsystem {
    public final static int DEFAULT_PIPELINE = 0;
    public final static int NONSENSE_VALUE = -4910;
    public final static double CAMERA_DH = 1.87; // Difference in meters between camera and center of goal
    public final static double CAMERA_ANGLE = 0.0; // Degrees

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
        targetTracker_ = new TargetTracker(this);
    }

    public static class LimelightRawData {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public int tWidth;
        public int tHeight;
        public double area;
        public boolean hasTarget;
        public double[] corners;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    private LimelightConfig config_ = null;
    private LimelightRawData rawData_ = new LimelightRawData();
    private boolean outputsHaveChanged_ = true;
    private TargetTracker targetTracker_;

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
        rawData_.latency = networkTable_.getEntry("tl").getDouble(0) / 1000.0 + Constants.IMAGE_CAPTURE_LATENCY;
        rawData_.givenLedMode = (int) networkTable_.getEntry("ledMode").getDouble(1.0);
        rawData_.givenPipeline = (int) networkTable_.getEntry("pipeline").getDouble(0);
        rawData_.xOffset = networkTable_.getEntry("tx").getDouble(0.0);
        rawData_.yOffset = networkTable_.getEntry("ty").getDouble(0.0);
        rawData_.tWidth = (int)networkTable_.getEntry("thor").getDouble(0.0);
        rawData_.tHeight = (int)networkTable_.getEntry("tvert").getDouble(0.0);
        rawData_.area = networkTable_.getEntry("ta").getDouble(0.0);
        rawData_.hasTarget = networkTable_.getEntry("tv").getDouble(0) == 1.0;
        double[] emptyArray = {};
        rawData_.corners = networkTable_.getEntry("tcornxy").getDoubleArray(emptyArray);
        targetTracker_.addLatestTargetData();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (rawData_.givenLedMode != rawData_.ledMode || rawData_.givenPipeline != rawData_.pipeline) {
            Logger.logDebug("Table has changed from expected, retrigger!!");
            outputsHaveChanged_ = true;
        }
        if (outputsHaveChanged_) {
            networkTable_.getEntry("ledMode").setNumber(rawData_.ledMode);
            networkTable_.getEntry("camMode").setNumber(rawData_.camMode);
            networkTable_.getEntry("pipeline").setNumber(rawData_.pipeline);
            networkTable_.getEntry("stream").setNumber(rawData_.stream);
            networkTable_.getEntry("snapshot").setNumber(rawData_.snapshot);

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
        TargetTracker.Reading currentReading = targetTracker_.getCurrentReading();
        SmartDashboard.putNumber("Azimuth: ", currentReading.azimuth);
        SmartDashboard.putNumber("Elevation: ", currentReading.elevation);
        SmartDashboard.putNumber("Range: ", currentReading.range);
        SmartDashboard.putNumber("Range Area: ", currentReading.rangeArea);
        SmartDashboard.putNumber("Range Corner: ", currentReading.rangeCorner);
        SmartDashboard.putNumber("Confidence: ", currentReading.confidence);
        SmartDashboard.putBoolean(config_.name + ": Has Target", rawData_.hasTarget);
        SmartDashboard.putNumber(config_.name + ": Pipeline Latency (ms)", rawData_.latency);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != rawData_.ledMode) {
            rawData_.ledMode = mode.ordinal();
            outputsHaveChanged_= true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != rawData_.pipeline) {
            rawData_.pipeline = mode;

            System.out.println(rawData_.pipeline + ", " + mode);
            outputsHaveChanged_ = true;
        }
    }

    public synchronized void triggerOutputs() {
        outputsHaveChanged_ = true;
    }

    public synchronized int getPipeline() {
        return rawData_.pipeline;
    }

    public synchronized boolean hasTarget() {
        return rawData_.hasTarget;
    }

    public synchronized double getTargetX() {
        return rawData_.xOffset;
    }

    public synchronized double getTargetY() {
        return rawData_.yOffset;
    }

    public synchronized boolean getTargetValid() {
        return rawData_.hasTarget;
    }

    public synchronized LimelightRawData getRawData() {
        return rawData_;
    }

    public double getLatency() {
        return rawData_.latency;
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
