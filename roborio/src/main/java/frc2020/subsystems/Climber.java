package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc2020.loops.ILooper;

public class Climber extends SingleMotorSubsystem {

    private static Climber instance_;

    private DoubleSolenoid armFlipper_;
    private final static int ARM_FLIPPER_PCM_ID = 1;
    private final static int ARM_FLIPPER_FORWARD_PORT = 0;
    private final static int ARM_FLIPPER_REVERSE_PORT = 1;
    private DoubleSolenoid winchLock_;
    private final static int WINCH_LOCK_PCM_ID = 0;
    private final static int WINCH_LOCK_FORWARD_PORT = 0;
    private final static int WINCH_LOCK_REVERSE_PORT = 1;

    private final static DoubleSolenoid.Value STOWED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value DEPLOYED_VALUE = Value.kForward;
    private final static DoubleSolenoid.Value UNLOCKED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value LOCKED_VALUE = Value.kForward;

    private static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
        new SingleMotorSubsystemConstants();
    static {
        MotorConstants masterConstants = new MotorConstants();
        masterConstants.id_ = 12;
        masterConstants.invertMotor_ = false;

        MotorConstants[] slaveConstantsArray = new MotorConstants[1];
        var slaveConstants = new MotorConstants();
        slaveConstants.id_ = 13;
        slaveConstants.invertMotor_ = true;
        slaveConstantsArray[0] = slaveConstants;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.slaveConstants_ = slaveConstantsArray;
        DEFAULT_CONSTANTS.name_ = "Climber";
    }

    public static Climber getInstance() {
        return instance_ == null ? instance_ = new Climber(DEFAULT_CONSTANTS) : instance_;
    }

    protected Climber(SingleMotorSubsystemConstants constants) {
        super(constants);

        armFlipper_ = new DoubleSolenoid(ARM_FLIPPER_PCM_ID, ARM_FLIPPER_FORWARD_PORT, ARM_FLIPPER_REVERSE_PORT);
        winchLock_ = new DoubleSolenoid(WINCH_LOCK_PCM_ID, WINCH_LOCK_FORWARD_PORT, WINCH_LOCK_REVERSE_PORT);
    }

    public void deployClimber() {
        armFlipper_.set()
    }

    @Override
    public boolean runActiveTests() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub

    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        // TODO Auto-generated method stub

    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

    @Override
    protected boolean atReverseLimit() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected boolean atForwardLimit() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected boolean handleZeroing() {
        // TODO Auto-generated method stub
        return false;
    }

}