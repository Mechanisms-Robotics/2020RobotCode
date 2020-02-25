package frc2020.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends SingleMotorSubsystem {

    private static Feeder instance_;

    private final static int INTAKE_SPEED = 3500; // rpm
    private final static int OUTTAKE_SPEED = -3500; // rpm
    private final static int PRIME_SPEED = -2000; // rpm TODO: Tune value
    private final static int SHOOTING_SPEED = 3500; // rpm TODO: Tune value

    private FeederState state_ = FeederState.IDLE;

    public enum FeederState {
        MANUAL,
        IDLE,
        INTAKING,
        PRIMING,
        SHOOTING
    }


    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS = 
        new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 5;
        masterConstants.invertMotor_ = false;
        masterConstants.invertSensorPhase_ = false;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.name_ = "Feeder";
        DEFAULT_CONSTANTS.enableHardLimits_ = false;
        DEFAULT_CONSTANTS.velocityKp_ = 0.00005;
        DEFAULT_CONSTANTS.velocityKi_ = 0.0;
        DEFAULT_CONSTANTS.velocityKd_ = 0.0;
        DEFAULT_CONSTANTS.velocityKf_ = 0.00017;
    }

    public static Feeder getInstance() {
        return instance_ == null ? instance_ = new Feeder(DEFAULT_CONSTANTS) : instance_;
    }

    public FeederState getState() {
        return state_;
    }

    public void setState(FeederState desiredState) {
        state_ = desiredState;
    }

    public void runFeeder(boolean outtake) {
        runFeeder(outtake ? OUTTAKE_SPEED : INTAKE_SPEED);
    }

    /**
     * @param speed in rpm
     */
    public void runFeeder(int speed) {
        super.setVelocity(speed);
    }

    protected Feeder(SingleMotorSubsystemConstants constants) {
        super(constants);
    }

    //TODO: Check what boolean is when broken
    public synchronized boolean getIntakeBreakBeamBroken() {
        return super.io_.forwardLimit;
    }
    
    //TODO: Check what boolean is when broken
    public synchronized boolean getShooterBreakBeamBroken() {
        return super.io_.reverseLimit;
    }

    @Override
    public boolean runActiveTests() {
        boolean hasPassedTests = true;
        logger_.logInfo("Starting feeder active tests", super.logName_);
        if(getIntakeBreakBeamBroken()) {
            hasPassedTests = false;
            logger_.logWarning("Intake break beams are broken", super.logName_);
        }
        if(getShooterBreakBeamBroken()) {
            hasPassedTests = false;
            logger_.logWarning("Shooter break beams are broken", super.logName_);
        }

        logger_.logInfo("Running feeder intake");
        super.setVelocity(INTAKE_SPEED);
        Timer.delay(1.5);
        super.stop();

        logger_.logInfo("Running feeder outtake");
        super.setVelocity(OUTTAKE_SPEED);
        Timer.delay(1.5);
        super.stop();

        return hasPassedTests;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();

        switch (state_) {
            case MANUAL:
                break;
            case IDLE:
                super.stop();
                break;
            case INTAKING:
                intakeFeeder();
                break;
            case PRIMING:
                primeFeeder();
                break;
            case SHOOTING:
                shootFeeder();
                break;
            default:
                logger_.logWarning("Invalid feeder state", logName_);
        }
    }

    public boolean isPrimed() {
        return true;
        /*
        if (state_ == FeederState.PRIMING) {
            return !turretBreakBeam_.get();
        }
        return false;*/
    }

    private synchronized void intakeFeeder() {
        if (getIntakeBreakBeamBroken()) {//} && !getShooterBreakBeamBroken()) {
            runFeeder(false);
        } else {
            super.stop();
        }
    }

    private synchronized void primeFeeder() {
        super.stop();
        /*
        if (turretBreakBeam_.get() && !intakeBreakBeam_.get()) {
            runFeeder(PRIME_SPEED);
        } else {
            super.stop();
        }*/
    }

    private synchronized void shootFeeder() {
        runFeeder(SHOOTING_SPEED);
    }

    @Override
    public void zeroSensors() {
        // Break beams are digital so no sensors to zero
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        // SmartDashboard.putBoolean("Intake Break Beam Broken", getIntakeBreakBeamBroken());
        // SmartDashboard.putBoolean("Shooter Break Beam Broken", getShooterBreakBeamBroken());
    }

    /**
     * No reverse limit for feeder
     */
    @Override
    protected boolean atReverseLimit() {
        return false;
    }

    /**
     * No forward limit for feeder
     */
    @Override
    protected boolean atForwardLimit() {
        return false;
    }

    @Override
    protected boolean handleZeroing() {
        return true;
    }
}
