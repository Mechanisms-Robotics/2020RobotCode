package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.util.Logger;

public class Intake extends SingleMotorSubsystem {

    private static Intake instance_;

    private final static int FLIPPER_FORWARD_PORT = 0; //TODO: Change for actual robot ports
    private final static int FLIPPER_REVERSE_PORT = 1;
    private final static DoubleSolenoid.Value STOWED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value DEPLOYED_VALUE = Value.kForward;
    private final static double INTAKE_RPM = 3200;
    private final static double REVERSE_RPM = -3200;
    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
        new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 3;
        masterConstants.invertMotor_ = false;
        
        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.name_ = "Intake";
    }

    private DoubleSolenoid flipper_;
    private boolean wantDeploy_ = false;
    private boolean isDeployed_ = false;

    private Logger logger_ = Logger.getInstance();
    private String logName;

    public static Intake getInstance() {
        return instance_ == null ? instance_ = new Intake(DEFAULT_CONSTANTS) : instance_;
    }

    protected Intake(SingleMotorSubsystemConstants constants) {
        super(constants);

        flipper_ = new DoubleSolenoid(FLIPPER_FORWARD_PORT, FLIPPER_REVERSE_PORT);
        logName = constants.name_;
    }

    public void deployIntake() {
        wantDeploy_ = true;
    }

    public void stowIntake() {
        super.stop();
        wantDeploy_ = false;
    }

    public void toggleIntake() {
        wantDeploy_ = !wantDeploy_;
    }

    /** 
     * Runs the intake in the given direction
     * 
     * @param reverse true if the intake is spinning such that 
     * we are intaking from above or outtaking from below
     */
    public void runIntake(boolean reverse) {
        if (reverse) {
            super.setVelocity(REVERSE_RPM);
        } else {
            super.setVelocity(INTAKE_RPM);
        }
    }

    /** 
     * Runs the active tests for the intake.
     * Deploys the intake, runs in both directions, then stows.
     * 
     * @return true always. This is a visual test
     */
    @Override
    public boolean runActiveTests() {
        logger_.logInfo("Running intake active tests", logName);

        logger_.logInfo("Deploying intake", logName);
        deployIntake();
        Timer.delay(1);
        
        logger_.logInfo("Intaking", logName);
        runIntake(false);
        Timer.delay(1.5);
        super.stop();

        logger_.logInfo("Outtaking", logName);
        runIntake(true);
        Timer.delay(1.5);
        super.stop();

        logger_.logInfo("Stowing intake", logName);
        stowIntake();

        return true;
    }

    @Override
    public void zeroSensors() {
        // No sensors to implement
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        // No loops to register
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Intake Deployed", isDeployed_);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();

        if (wantDeploy_ != isDeployed_) {
            if (wantDeploy_) {
                flipper_.set(DEPLOYED_VALUE);
            } else {
                flipper_.set(STOWED_VALUE);
            }
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();

        isDeployed_ = flipper_.get() == DEPLOYED_VALUE;
    }

    @Override
    protected boolean atReverseLimit() {
        return !isDeployed_;
    }

    @Override
    protected boolean atForwardLimit() {
        return !isDeployed_;
    }

}