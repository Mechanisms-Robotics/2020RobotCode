package frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Feeder subsystem class for our robot. Implements SingleMotorSubsystem
 * because feeder runs on only one motor.
 */
public class Feeder extends SingleMotorSubsystem {

    private static Feeder instance_;

    private final static int INTAKE_SPEED = 3500; // rpm
    private final static int OUTTAKE_SPEED = -3500; // rpm
    private final static int PRIME_SPEED = -2000; // rpm TODO: Tune value
    private final static int SHOOTING_SPEED = 3500; // rpm TODO: Tune value

    private final static int INTAKE_BREAK_BEAM_CHANNEL = 0; // TODO: Change for robot	
    private final static int TURRET_BREAK_BEAM_CHANNEL = 1; // TODO: Change for robot	

    private DigitalInput intakeBreakBeam_;	
    private DigitalInput turretBreakBeam_;

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

    /**
     * @return feeder instance
     */
    public static Feeder getInstance() {
        return instance_ == null ? instance_ = new Feeder(DEFAULT_CONSTANTS) : instance_;
    }

    /**
     * @return gets feeder state [MANUAL, IDLE, INTAKING, PRIMING, SHOOTING]
     */
    public FeederState getState() {
        return state_;
    }

    /**
     * set our feeder state [MANUAL, IDLE, INTAKING, PRIMING, SHOOTING]
     */
    public void setState(FeederState desiredState) {
        state_ = desiredState;
    }

    /**
     * main function for intaking/outtaking balls through feeder
     * @param outtake runs feeder in reverse
     */
    public void runFeeder(boolean outtake) {
        runFeeder(outtake ? OUTTAKE_SPEED : INTAKE_SPEED);
    }

    /**
     * @param speed in rpm
     */
    public void runFeeder(int speed) {
        super.setVelocity(speed);
    }

    /**
     * constructs intake and shooter break beams
     */
    protected Feeder(SingleMotorSubsystemConstants constants) {
        super(constants);
        
        intakeBreakBeam_ = new DigitalInput(INTAKE_BREAK_BEAM_CHANNEL);	
        turretBreakBeam_ = new DigitalInput(TURRET_BREAK_BEAM_CHANNEL);
    }

    /**
     * @return true if break beam is broken
     */
    //TODO: Check what boolean is when broken
    public synchronized boolean getIntakeBreakBeamBroken() {
        return !intakeBreakBeam_.get();
    }

    /**
     * @return true if break beam is broken
     */
    //TODO: Check what boolean is when broken
    public synchronized boolean getShooterBreakBeamBroken() {
        return !turretBreakBeam_.get();
    }

    /**
     * If either break beams are broken test fails, and feeder
     * intakes and outtake for 1.5 sec each
     * @return true if feeder has passed all active tests
     */
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

    /**
     * Calls functions for feeder depending on feeder state
     */
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

    /**
     * @return true if feeder is primed
     */
    public boolean isPrimed() {
        if (state_ == FeederState.PRIMING) {
            return !getShooterBreakBeamBroken();
        }
        return false;
    }

    /**
     *
     */
    private synchronized void intakeFeeder() {
        if (getIntakeBreakBeamBroken()) {//&& !getShooterBreakBeamBroken()) {
            runFeeder(false);
        } else {
            super.stop();
        }
    }

    private synchronized void primeFeeder() {
        if (getShooterBreakBeamBroken()) {
            runFeeder(PRIME_SPEED);
        } else {
            super.stop();
        }
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
        SmartDashboard.putBoolean("Intake Break Beam Broken", getIntakeBreakBeamBroken());
        SmartDashboard.putBoolean("Shooter Break Beam Broken", getShooterBreakBeamBroken());
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
