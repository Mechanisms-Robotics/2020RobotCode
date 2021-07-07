package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;

public class Intake extends SingleMotorSubsystem {

    private static Intake instance_;

    private final static int FLIPPER_FORWARD_PORT = 4;
    private final static int FLIPPER_REVERSE_PORT = 5;
    private final static DoubleSolenoid.Value STOWED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value DEPLOYED_VALUE = Value.kForward;
    private final static double INTAKE_RPM = 4000; // 5600 comp bot before
    private final static double REVERSE_RPM = -INTAKE_RPM;
    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
        new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 9;
        masterConstants.invertMotor_ = true;
        // TODO: Tune these
        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.name_ = "Intake";
        DEFAULT_CONSTANTS.velocityKp_ = 0.000045;
        DEFAULT_CONSTANTS.velocityKi_ = 0.0;
        DEFAULT_CONSTANTS.velocityKd_ = 0.0;
        DEFAULT_CONSTANTS.velocityKf_ = Constants.IS_COMP_BOT ?  0.00009 : 0.0001;
    }

    private DoubleSolenoid flipper_;
    private boolean wantDeploy_ = false;
    private boolean isDeployed_ = false;

    public static Intake getInstance() {
        return instance_ == null ? instance_ = new Intake(DEFAULT_CONSTANTS) : instance_;
    }

    protected Intake(SingleMotorSubsystemConstants constants) {
        super(constants);
        flipper_ = new DoubleSolenoid(FLIPPER_FORWARD_PORT, FLIPPER_REVERSE_PORT);
    }

    public void deployIntake() {
        wantDeploy_ = true;
    }

    public void stowIntake() {
        wantDeploy_ = false;
    }

    public void toggleIntake() {
        wantDeploy_ = !wantDeploy_;
        if (!wantDeploy_) {
            super.stop();
        }
    }

    /** 
     * Runs the intake in the given direction
     * 
     * @param reverse true if the intake is spinning such that 
     * we are intaking from above or outtaking from below
     */
    public void runIntake(boolean reverse) {
        // TODO: Tune percentage then find RPM
        if (reverse) {
            //runIntake(REVERSE_RPM);
            super.setOpenLoop(-0.7);
        } else {
            //runIntake(INTAKE_RPM);
            super.setOpenLoop(0.7);
        }
    }

    public void runIntake(double speed) {
        super.setVelocity(speed);
    }

    /** 
     * Runs the active tests for the intake.
     * Deploys the intake, runs in both directions, then stows.
     * 
     * @return true always. This is a visual test
     */
    @Override
    public boolean runActiveTests() {
        logger_.logInfo("Running intake active tests", super.logName_);

        logger_.logInfo("Deploying intake", super.logName_);
        deployIntake();
        Timer.delay(1);
        
        logger_.logInfo("Intaking", super.logName_);
        runIntake(false);
        Timer.delay(1.5);

        logger_.logInfo("Outtaking", super.logName_);
        runIntake(true);
        Timer.delay(1.5);

        logger_.logInfo("Stowing intake", super.logName_);
        stowIntake();

        return true;
    }

    @Override
    public void zeroSensors() {
        // No sensors to implement
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
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
        return false;
    }

    @Override
    protected boolean atForwardLimit() {
        return false;
    }

    @Override
    protected boolean handleZeroing() {
        return true;
    }

}