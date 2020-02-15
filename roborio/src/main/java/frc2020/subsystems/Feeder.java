package frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.util.Logger;

public class Feeder extends SingleMotorSubsystem {

    private static Feeder instance_;

    private final static int INTAKE_BREAK_BEAM = 1; // TODO: Check actual robot ports
    private final static int SHOOTER_BREAK_BEAM = 2;

    private DigitalInput intakeBreakBeam_;
    private DigitalInput shooterBreakBeam_;

    private Logger logger_ = Logger.getInstance();

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS = 
        new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 4;
        masterConstants.invertMotor_ = false;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.name_ = "Feeder";
    }

    public static Feeder getInstance() {
        return instance_ == null ? instance_ = new Feeder(DEFAULT_CONSTANTS) : instance_;
    }

    protected Feeder(SingleMotorSubsystemConstants constants) {
        super(constants);
        intakeBreakBeam_ = new DigitalInput(INTAKE_BREAK_BEAM);
        shooterBreakBeam_ = new DigitalInput(SHOOTER_BREAK_BEAM);
    }

    //TODO: Check what boolean is when broken
    public synchronized boolean getIntakeBreakBeamBroken() {
        return intakeBreakBeam_.get();
    }
    
    //TODO: Check what boolean is when broken
    public synchronized boolean getShooterBreakBeamBroken() {
        return shooterBreakBeam_.get();
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
        super.setVelocity(3000); //TODO: Adjust velocity once we can test speed
        Timer.delay(1.5);
        super.stop();

        logger_.logInfo("Running feeder outtake");
        super.setVelocity(-3000); //TODO: Adjust velocity once we can test speed
        Timer.delay(1.5);
        super.stop();

        return hasPassedTests;
    }

    @Override
    public void zeroSensors() {
        // Break beams are digital so no sensors to zero
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        // TODO: Determine if needed

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