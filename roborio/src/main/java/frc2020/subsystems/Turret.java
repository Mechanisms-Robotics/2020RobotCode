package frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.loops.ILooper;
import frc2020.util.Logger;

public class Turret extends SingleMotorSubsystem {

    // TODO: Measure physical position of limits.
    private final static double LEFT_LIMIT_POS_POSITIVE = 140;
    private final static double RIGHT_LIMIT_POS_NEGATIVE = -140;
    private final static double LEFT_LIMIT_POS_NEGATIVE = LEFT_LIMIT_POS_POSITIVE - 360;
    private final static double RIGHT_LIMIT_POS_POSITIVE = RIGHT_LIMIT_POS_NEGATIVE + 360;
    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
            new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 6;
        masterConstants.invertMotor_ = false;
        masterConstants.invertSensorPhase_ = false;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.homePosition_ = 0.0; //degrees
        DEFAULT_CONSTANTS.name_ = "Turret";
    }

    private final static Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(180);

    private boolean leftLimit = false;
    private boolean rightLimit = false;

    private Logger logger_ = Logger.getInstance();

    protected Turret(SingleMotorSubsystemConstants constants) {
        super(constants);
    }

    @Override
    protected boolean handleZeroing() {
        if (leftLimit) {
            if (getPosition() >= 0) {
                encoder.setPosition(LEFT_LIMIT_POS_POSITIVE);
            } else {
                encoder.setPosition(LEFT_LIMIT_POS_NEGATIVE);
            }
            return true;
        } else if (rightLimit) {
            if (getPosition() >= 0) {
                encoder.setPosition(RIGHT_LIMIT_POS_POSITIVE);
            } else {
                encoder.setPosition(RIGHT_LIMIT_POS_NEGATIVE);
            }
            return true;
        }
        return false;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        // TODO: Set left and right limit
    }

    public synchronized boolean atLeftLimit() {
        return leftLimit;
    }

    public synchronized boolean atRightLimit() {
        return rightLimit;
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected synchronized boolean atReverseLimit() {
        // This is a safety. If the robot starts with the turret pointed in the
        // wrong direction, this will prevent us from ripping out the chain.
        if (!hasBeenZeroed) {
            return io_.velocity <= 0.0 && leftLimit;
        }

        // If were are here, the robot has been zeroed
        return encoder.getPosition() <= 0 && leftLimit;
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected synchronized boolean atForwardLimit() {
        // This is a safety. If the robot starts with the turret pointed in the
        // wrong direction, this will prevent us from ripping out the chain.
        if (!hasBeenZeroed) {
            return io_.velocity >= 0.0 && rightLimit;
        }

        // If were are here, the robot has been zeroed
        return encoder.getPosition() >= 0 && rightLimit;
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

    public synchronized void setAbsolutePosition(Rotation2d position) {

        // If the demand is outside the zone of no ambiguity then we can just go there
        if (position.getDegrees() < LEFT_LIMIT_POS_POSITIVE && position.getDegrees() > RIGHT_LIMIT_POS_NEGATIVE) {
            setSmartPosition(position.getDegrees());
            return;
        }

        // If we we can get to the setpoint by going either counter-clockwise or clockwise
        // we check to see what is the closest way to get there and do that.
        // This means that if we are at a positive position then it's faster
        // to turn counter-clockwise. Otherwise turn clockwise.
        double delta = position.minus(getRotation()).getDegrees();
        setSmartPosition(getPosition() + delta);
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

    }
}
