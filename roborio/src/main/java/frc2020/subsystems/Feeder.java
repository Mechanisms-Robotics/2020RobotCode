package frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Feeder extends SingleMotorSubsystem {

    private static Feeder instance_;

    public final static int INTAKE_SPEED = 4500; // rpm // changed from 3500
    private final static int OUTTAKE_SPEED = -4500; // rpm // changed from 3500
    private final static int PRIME_SPEED = -2000; // rpm 
    private final static int SHOOTING_SPEED = 5500; // rpm

    private final static int INTAKE_BREAK_BEAM_CHANNEL = 0;	
    private final static int TURRET_BREAK_BEAM_CHANNEL = 1;	

    private boolean overrideIntakeBreakBeam_ = false;

    private DigitalInput intakeBreakBeam_;	
    private DigitalInput turretBreakBeam_;

    private FeederState state_ = FeederState.IDLE;

    private Timer feederCycleTimer;
    private boolean feederCycleTimerReset = false;
    private static final double feederCycleTimeDelay = 0.1; // seconds
    private boolean flipDirection = false;

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
        runFeeder(outtake ? -SHOOTING_SPEED : SHOOTING_SPEED);
    }

    /**
     * @param speed in rpm
     */
    public void runFeeder(int speed) {
        super.setVelocity(speed);
    }

    protected Feeder(SingleMotorSubsystemConstants constants) {
        super(constants);

        intakeBreakBeam_ = new DigitalInput(INTAKE_BREAK_BEAM_CHANNEL);
        turretBreakBeam_ = new DigitalInput(TURRET_BREAK_BEAM_CHANNEL);

        feederCycleTimer = new Timer();
    }

    public void setOverrideIntakeBreakBeam(boolean overrideIntakeBreakBeam) { overrideIntakeBreakBeam_ = overrideIntakeBreakBeam; }

    public synchronized boolean getIntakeBreakBeamBroken() {
        return !intakeBreakBeam_.get();
    }
    
    public synchronized boolean getShooterBreakBeamBroken() {
        return !turretBreakBeam_.get();
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

    public boolean getOverrideIntakeBrakeBeam() { return overrideIntakeBreakBeam_; }

    public boolean isPrimed() {
        if (state_ == FeederState.PRIMING) {
            return !getShooterBreakBeamBroken();
        }
        return false;
    }

    private synchronized void intakeFeeder() {
        if (getIntakeBreakBeamBroken() && !overrideIntakeBreakBeam_) {
            if (feederCycleTimerReset) {
                feederCycleTimer.stop();
                feederCycleTimer.reset();
                feederCycleTimerReset = false;
                System.out.println("reset");
            }
            feederCycleTimer.start();
            if (feederCycleTimer.hasElapsed(feederCycleTimeDelay)) {
                System.out.println("flip");
                feederCycleTimerReset = true;
                flipDirection = !flipDirection;
            }
            if (flipDirection) {
                super.stop();
            } else {
                runFeeder(false);
            }
            return;
        } else if (!overrideIntakeBreakBeam_) {
            super.stop();
        }
        feederCycleTimerReset = true;
        flipDirection = false;
    }

    private synchronized void primeFeeder() {
        if (getShooterBreakBeamBroken()) {
            runFeeder(PRIME_SPEED);
        } else {
            super.stop();
        }
    }

    public synchronized void shootFeeder() {
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
        SmartDashboard.putBoolean("Intake Break Beams Overriden", getOverrideIntakeBrakeBeam());
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
