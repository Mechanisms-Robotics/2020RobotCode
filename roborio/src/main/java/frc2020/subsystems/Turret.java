package frc2020.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.loops.ILooper;

public class Turret extends SingleMotorSubsystem {

    // Notes about sensor vs turret units
    // Turret units has the back of the robot as 0
    // Sensor units has the reverse limit switch as 0
    // This makes measurements and calibration much simplier

    private static Turret instance_;

    // TODO: Measure
    private final static Rotation2d TURRET_HOME_TO_SENSOR_HOME = Rotation2d.fromDegrees(0.0);
    private final static double FORWARD_LIMIT_SWITCH_POSITION = 355;
    private final static double REVERSE_LIMIT_SWITCH_POSITION = 0;

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
            new SingleMotorSubsystemConstants();

    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 7;
        masterConstants.invertMotor_ = false;
        masterConstants.invertSensorPhase_ = false;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.homePosition_ = 0.0; //degrees
        DEFAULT_CONSTANTS.name_ = "Turret";
        DEFAULT_CONSTANTS.enableHardLimits_ = false;

        DEFAULT_CONSTANTS.positionConversionFactor_ = 3.3; // degrees / encoder
        DEFAULT_CONSTANTS.velocityConversionFactor_ = 3.3;
        DEFAULT_CONSTANTS.closedLoopRampRate_ = 0.1;
        DEFAULT_CONSTANTS.cruiseVelocity_ = 0; //deg/min
        DEFAULT_CONSTANTS.acceleration_ = 0; //deg/min^2
        DEFAULT_CONSTANTS.velocityDeadBand_ = 2.5;

        DEFAULT_CONSTANTS.kP_ = 0.0001;
        DEFAULT_CONSTANTS.kI_ = 0.0;
        DEFAULT_CONSTANTS.kD_ = 0.0;
        DEFAULT_CONSTANTS.kF_ = 0.0;
    }

    private final static Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(180);

    public static Turret getInstance() {
        return instance_ == null ? instance_ = new Turret(DEFAULT_CONSTANTS) : instance_;
    }

    protected Turret(SingleMotorSubsystemConstants constants) {
        super(constants);
    }

    /**
     * Set's the poseidon of the turret given a change in rotation.
     * @param deltaRotation The rotation to change the turret angle by.
     *                      positive counter-clockwise
     */
    public synchronized void setRelativeRotation(Rotation2d deltaRotation) {
        setSmartPosition(getPosition() + deltaRotation.getDegrees());
    }

    /**
     * The absolute position to set the turret's angle to given an absolute
     * rotation
     * @param absolutePosition The absolute rotation to turn to. Positive counter-clockwise
     *                         0 is the turret facing the back of the robot.
     */
    public synchronized void setAbsoluteRotation(Rotation2d absolutePosition) {
        setSmartPosition(toTurretSetpoint(absolutePosition));
    }

    private static double toTurretSetpoint(Rotation2d setpoint) {
        double correctedRotation = setpoint.rotateBy(TURRET_HOME_TO_SENSOR_HOME).getDegrees();
        if (correctedRotation < 0.0) {
            correctedRotation += 360.0;
        }
        return correctedRotation;
    }

    @Override
    protected boolean handleZeroing() {
        // TODO: Enable zeroing once sensor units and limit swich posotion is found
        final boolean enableZeroing = false;
        if (enableZeroing) {
            if (atForwardLimit()) {
                encoder.setPosition(FORWARD_LIMIT_SWITCH_POSITION);
                return true;
            }
            if (atReverseLimit()) {
                encoder.setPosition(REVERSE_LIMIT_SWITCH_POSITION);
                return true;
            }
        }
        return false;
    }

    @Override
    protected synchronized boolean atReverseLimit() {
        return io_.reverseLimit;
    }

    @Override
    protected synchronized boolean atForwardLimit() {
        return io_.forwardLimit;
    }

    @Override
    public boolean runActiveTests() {
        return true;
    }

    @Override
    public void zeroSensors() {
        encoder.setPosition(0.0);
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
    }
}
