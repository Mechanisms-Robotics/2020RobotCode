package frc2020.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;

public class Feeder extends SingleMotorSubsystem {

    private static Feeder instance_;

    private final static int INTAKE_SPEED = 5000; // rpm
    private final static int OUTTAKE_SPEED = -5000; // rpm

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
    public void zeroSensors() {
        // Break beams are digital so no sensors to zero
    }

    @Override
    public void outputTelemetry() {
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
