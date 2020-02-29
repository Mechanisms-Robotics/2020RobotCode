package frc2020.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;
import frc2020.util.DriveSignal;
import frc2020.util.Logger;

/**
 * This class controls our Climber subsystem of our robot. Climber implements
 * subsystem rather than SingleMotorSubsystem because winches want to have the
 * option to be independently controlled rather than one slaved off of the other.
 */
public class Climber implements Subsystem {

    private static Climber instance_;

    private boolean hasDeployed_ = false; //This is set to true ONCE after deploy

    private DoubleSolenoid armFlipper_;
    private final static int ARM_FLIPPER_PCM_ID = 1;
    private final static int ARM_FLIPPER_FORWARD_PORT = 0;
    private final static int ARM_FLIPPER_REVERSE_PORT = 1;
    private boolean wantDeploy_ = false;
    private boolean isDeployed_ = false;

    private DoubleSolenoid winchLock_;
    private final static int WINCH_LOCK_PCM_ID = 0;
    private final static int WINCH_LOCK_FORWARD_PORT = 0;
    private final static int WINCH_LOCK_REVERSE_PORT = 1;
    private boolean wantLock_ = false;
    private boolean isLocked_ = false;

    private CANSparkMax leftClimb_;
    private CANSparkMax rightClimb_;

    private CANPIDController leftClimbPID;
    private CANPIDController rightClimbPID;

    private CANEncoder rightEncoder;
    private CANEncoder leftEncoder;

    private static final int STALL_LIMIT = 30; // amps
    private static final int FREE_LIMIT = 30; // amps

    private PeriodicIO io_ = new PeriodicIO();

    private final static DoubleSolenoid.Value STOWED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value DEPLOYED_VALUE = Value.kForward;
    private final static DoubleSolenoid.Value UNLOCKED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value LOCKED_VALUE = Value.kForward;

    private Logger logger_ = Logger.getInstance();
    private String logName = "Climber";

    /**
     * Configures our spark maxes for the climb winches in specific. Just
     * bare basic settings for the sparks because we don't need to do any
     * complex tasks with these.
     */
    private void configSparkMaxs() {
        leftClimb_ = new CANSparkMax(Constants.LEFT_CLIMB_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightClimb_ = new CANSparkMax(Constants.RIGHT_CLIMB_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftClimb_.restoreFactoryDefaults();
        rightClimb_.restoreFactoryDefaults();

        leftClimb_.setInverted(true);
        rightClimb_.setInverted(false);

        leftClimb_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10);
        rightClimb_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10);
        leftClimb_.setSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT);
        rightClimb_.setSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT);
    }

    /**
     * @return climber instance
     */
    public static Climber getInstance() {
        return instance_ == null ? instance_ = new Climber() : instance_;
    }

    /**
     * Configures sparks and solenoids
     */
    protected Climber() {
        configSparkMaxs();
        leftClimbPID = leftClimb_.getPIDController();
        rightClimbPID = rightClimb_.getPIDController();
        leftEncoder = leftClimb_.getEncoder();
        rightEncoder = rightClimb_.getEncoder();
        armFlipper_ = new DoubleSolenoid(ARM_FLIPPER_PCM_ID, ARM_FLIPPER_FORWARD_PORT, ARM_FLIPPER_REVERSE_PORT);
        winchLock_ = new DoubleSolenoid(WINCH_LOCK_PCM_ID, WINCH_LOCK_FORWARD_PORT, WINCH_LOCK_REVERSE_PORT);
    }

    /**
     * Deploys climber solenoids
     */
    public void deployClimberArm() {
        wantDeploy_ = true;
    }

    /**
     * Stows climber arms and if hasn't deployed set winch motors to 0 for safety.
     * If it has deployed we can move motors for extra pulling power on shield
     * generator.
     */
    public void stowClimberArm() {
        if(!hasDeployed_) {
            setOpenLoop(new DriveSignal(0.0, 0.0));
        }
        wantDeploy_ = false;
    }

    /**
     * Locks winch
     */
    public void lockWinch() {
        wantLock_ = true;
    }

    /**
     * Unlocks winch
     */
    public void unlockWinch() {
        wantLock_ = false;
    }

    /**
     * Powers motors using an openloop drive signal demand
     */
    public void controlWinch(DriveSignal demand) {
        setOpenLoop(demand);
    }

    /**
     * Call between teleop runs for safety
     * Sets hasDeployed_ to false so winch does not run
     * until deployed
     */
    public void resetHasDeployed() {
        hasDeployed_ = false;
    }

    /**
     * simple openloop function that takes in a driveSignal
     */
    private void setOpenLoop(DriveSignal signal) {
        io_.leftDemand = signal.getLeft();
        io_.rightDemand = signal.getRight();
    }

    @Override
    public boolean runActiveTests() {
        return false;
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void registerLoops(ILooper enabledLooper) {

    }

    public static class PeriodicIO {
        // Inputs
        public double timestamp;
        public double rightVelocity;
        public double leftVelocity;
        public double rightOutputPercent;
        public double leftOutputPercent;
        public double rightOutputVoltage;
        public double leftOutputVoltage;
        public double rightMasterCurrent;
        public double leftMasterCurrent;
        public double leftDemand;
        public double rightDemand;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        io_.timestamp = Timer.getFPGATimestamp();
        io_.rightVelocity = rightEncoder.getVelocity();
        io_.leftVelocity = leftEncoder.getVelocity();
        io_.rightMasterCurrent = rightClimb_.getOutputCurrent();
        io_.leftMasterCurrent = leftClimb_.getOutputCurrent();
        io_.rightOutputPercent = rightClimb_.getAppliedOutput();
        io_.leftOutputPercent = leftClimb_.getAppliedOutput();
        io_.rightOutputVoltage = io_.rightOutputPercent * rightClimb_.getBusVoltage();
        io_.leftOutputVoltage = io_.leftOutputPercent * leftClimb_.getBusVoltage();

        isDeployed_ = armFlipper_.get() == DEPLOYED_VALUE;
        isLocked_ = winchLock_.get() == LOCKED_VALUE;
    }

    /**
     * writes arm solenoid deploys/retracts, if arms deployed,
     * and winch locks/unlocks
     */
    @Override
    public synchronized void writePeriodicOutputs() {
        if (wantDeploy_ != isDeployed_) {
            if (wantDeploy_) {
                armFlipper_.set(DEPLOYED_VALUE);
            } else {
                armFlipper_.set(STOWED_VALUE);
            }
        }

        if(isDeployed_) {
            hasDeployed_ = true;
        }

        if(wantLock_ != isLocked_) {
            if(wantLock_) {
                winchLock_.set(LOCKED_VALUE);
            } else {
                winchLock_.set(UNLOCKED_VALUE);
            }
        }

        rightClimbPID.setReference(io_.rightDemand, ControlType.kDutyCycle);
        leftClimbPID.setReference(io_.leftDemand, ControlType.kDutyCycle);
    }


    @Override
    public boolean runPassiveTests() {
        return true;
    }

    /**
     * Outputs whether climber is deployed and whether climber is locked
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Climber deployed", isDeployed_);
        SmartDashboard.putBoolean("Climber LOCKED", isLocked_);
    }
}
