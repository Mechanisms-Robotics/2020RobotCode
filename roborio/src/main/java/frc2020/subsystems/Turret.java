package frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;

public class Turret extends SingleMotorSubsystem {

    private static Turret instance_;

    // TODO: Measure physical position of limits.
    private final static double LEFT_LIMIT_POS_POSITIVE = 113;
    private final static double RIGHT_LIMIT_POS_NEGATIVE = -113;
    private final static double LEFT_LIMIT_POS_NEGATIVE = LEFT_LIMIT_POS_POSITIVE - 360;
    private final static double RIGHT_LIMIT_POS_POSITIVE = RIGHT_LIMIT_POS_NEGATIVE + 360;
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
        DEFAULT_CONSTANTS.cruiseVelocity_ = 12000; //deg/min
        DEFAULT_CONSTANTS.acceleration_ = 12000; //deg/min^2
        DEFAULT_CONSTANTS.velocityDeadBand_ = 2.5;

        DEFAULT_CONSTANTS.kP_ = 0.0001;
        DEFAULT_CONSTANTS.kI_ = 0.0;
        DEFAULT_CONSTANTS.kD_ = 0.0;
        DEFAULT_CONSTANTS.kF_ = 0.0;
    }

    private final static Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(180);

    private final DigitalInput leftLimit;
    private final DigitalInput rightLimit;

    public static Turret getInstance() {
        return instance_ == null ? instance_ = new Turret(DEFAULT_CONSTANTS) : instance_;
    }

    protected Turret(SingleMotorSubsystemConstants constants) {
        super(constants);
        leftLimit = new DigitalInput(0);
        rightLimit = new DigitalInput(1);
    }

    @Override
    protected boolean handleZeroing() {
        //logger_.logDebug("Calling handle zeroing", logName_);
        if (atLeftLimit()) {
            if (getPosition() >= 0) {
                encoder.setPosition(LEFT_LIMIT_POS_POSITIVE);
            } else {
                encoder.setPosition(LEFT_LIMIT_POS_NEGATIVE);
            }
            logger_.logInfo("Turret zeroed at left limit!", super.logName_);
            return true;
        } else if (atRightLimit()) {
            if (getPosition() >= 0) {
                encoder.setPosition(RIGHT_LIMIT_POS_POSITIVE);
            } else {
                encoder.setPosition(RIGHT_LIMIT_POS_NEGATIVE);
            }
            logger_.logInfo("Turret zeroed at right limit!", super.logName_);
            return true;
        }
        return false;
    }

    public synchronized boolean atLeftLimit() {
        return !leftLimit.get();
    }

    public synchronized boolean atRightLimit() {
        return !rightLimit.get();
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected synchronized boolean atReverseLimit() {
        // This is a safety. If the robot starts with the turret pointed in the
        // wrong direction, this will prevent us from ripping out the chain.
        if (!hasBeenZeroed) {
            return io_.velocity <= 0.0 && atLeftLimit();
        }

        // If were are here, the robot has been zeroed
        return encoder.getPosition() <= 0 && atLeftLimit();
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected synchronized boolean atForwardLimit() {
        // This is a safety. If the robot starts with the turret pointed in the
        // wrong direction, this will prevent us from ripping out the chain.
        if (!hasBeenZeroed) {
            return io_.velocity >= 0.0 && atRightLimit();
        }

        // If were are here, the robot has been zeroed
        return encoder.getPosition() >= 0 && atRightLimit();
    }

    /**
     * Get the rotation of the turret.
     * @return Gets the rotation of the turret in turret coordinates
     *         (i.e. 0 is towards the back of the robot.)
     */
    public synchronized Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getPosition());
    }

    /**
     * Get the rotation of the turret relative to the robot
     * @return Gets the rotation of the turret in robot coordinates
     *         (i.e. 0 is towards the front of the robot.)
     */
    public synchronized Rotation2d getTurretToRobot() {
        return getRotation().rotateBy(TURRET_TO_ROBOT);
    }

    /**
     * Sets the position of the turret in turret coordinates.
     * @param position The desired position of the turret in turret coordinates
     */
    public synchronized void setAbsolutePosition(Rotation2d position) {

        // If the demand is outside the zone of no ambiguity then we can just go there
        if (position.getDegrees() <= LEFT_LIMIT_POS_POSITIVE && position.getDegrees() >= RIGHT_LIMIT_POS_NEGATIVE) {
            setSmartPosition(position.getDegrees());
            return;
        }

        // If the demand is inside the zone of ambiguity then we calculate
        // the shortest distance the turret needs to travel to to the
        // goal position
        double distance = position.minus(getRotation()).getDegrees();
        setSmartPosition(getPosition() + distance);
    }

    /**
     * Rotates the position of the turret by delta angle
     * @param delta Delta relative to current turret angle
     */
    public synchronized void setRelativePosition(Rotation2d delta) {

        // Calculate current rotation rotated by delta
        logger_.logDebug("delta: " + delta + "  rotation: " + getRotation());
        Rotation2d relativeRotation = getRotation().rotateBy(delta);

        setAbsolutePosition(relativeRotation);
    }

    @Override
    public boolean runActiveTests() {
        if(!hasBeenZeroed) {
            logger_.logWarning("Cannot run Turret tests! NOT ZEROED!", super.logName_);
            return false;
        }

        boolean hasPassedTests = true;

        logger_.logInfo("Homing turret", super.logName_);

        double startTime = Timer.getFPGATimestamp();
        super.setSmartPosition(super.constants_.homePosition_);
        while(!super.atDemand()){
            if((Timer.getFPGATimestamp() - startTime) >= 1.5) {
                logger_.logWarning("Turret homing timed out!", super.logName_);
                hasPassedTests = false;
                break;
            }
            Timer.delay(0.1);
        }

        logger_.logInfo("Tracking to left limit", super.logName_);

        startTime = Timer.getFPGATimestamp();
        super.setSmartPosition(LEFT_LIMIT_POS_POSITIVE);
        while(!super.atDemand()){
            if((Timer.getFPGATimestamp() - startTime) >= 1.5) {
                logger_.logWarning("Turret tracking timed out!", super.logName_);
                hasPassedTests = false;
                break;
            }
            Timer.delay(0.1);
        }

        if (!atLeftLimit()) {
            logger_.logWarning("Left limit switch not hit!", super.logName_);
            hasPassedTests = false;
        }

        logger_.logInfo("Tracking to right limit", super.logName_);

        startTime = Timer.getFPGATimestamp();
        super.setSmartPosition(RIGHT_LIMIT_POS_NEGATIVE);
        while(!super.atDemand()){
            if((Timer.getFPGATimestamp() - startTime) >= 3.0) {
                logger_.logWarning("Turret tracking timed out!", super.logName_);
                hasPassedTests = false;
                break;
            }
            Timer.delay(0.1);
        }

        if(!atRightLimit()) {
            logger_.logWarning("Right limit switch not hit!", super.logName_);
            hasPassedTests = false;
        }

        logger_.logInfo("Re-homing turret", super.logName_);

        startTime = Timer.getFPGATimestamp();
        super.setSmartPosition(super.constants_.homePosition_);
        while(!super.atDemand()){
            if((Timer.getFPGATimestamp() - startTime) >= 1.5) {
                logger_.logWarning("Turret homing timed out!", super.logName_);
                hasPassedTests = false;
                break;
            }
            Timer.delay(0.1);
        }

        return hasPassedTests;
    }

    @Override
    public void zeroSensors() {
        encoder.setPosition(0.0);
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {

    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        SmartDashboard.putBoolean("Turret Left Limit", atLeftLimit());
        SmartDashboard.putBoolean("Turret Right Limit", atRightLimit());
    }
}
