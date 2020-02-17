package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc2020.loops.ILooper;

public class Hood extends SingleMotorSubsystem {

    private static Hood instance_;

    private final static int FLIPPER_FORWARD_PORT = 2;
    private final static int FLIPPER_REVERSE_PORT = 5;
    private final static DoubleSolenoid.Value STOWED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value DEPLOYED_VALUE = Value.kForward;

    /*TODO: when we have the robot, set this value to halfway between all the way back
            and at the forward position of the reverse limit switch
    */
    private final static int STOW_POSITION = 10; // encoder units

    private DoubleSolenoid flipper_;
    private boolean wantDeploy_ = false;
    private boolean isDeployed_ = false;

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
        new SingleMotorSubsystemConstants();

    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 8;
        masterConstants.invertMotor_ = false;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.name_ = "Hood";
        DEFAULT_CONSTANTS.enableHardLimits_ = true; //TODO: verify limit switch plugs are correct
    }

    protected Hood(SingleMotorSubsystemConstants constants) {
        super(constants);
        flipper_ = new DoubleSolenoid(FLIPPER_FORWARD_PORT, FLIPPER_REVERSE_PORT);
    }

    public static Hood getInstance() {
        return instance_ == null ? instance_ = new Hood(DEFAULT_CONSTANTS) : instance_;
    }

    public void deployHood() {
        wantDeploy_ = true;
    }

    public void stowHood() {
        setSmartPosition(STOW_POSITION);
        wantDeploy_ = false;
    }

    public void toggleHood() {
        if (wantDeploy_) {
            stowHood();
        } else {
            deployHood();
        }
    }

	@Override
    public void zeroSensors() { //TODO: Figure out when to call
        if (!hasBeenZeroed && atReverseLimit()) {
            encoder.setPosition(0.0);
            hasBeenZeroed = true;
        }
	}

	@Override
	public void registerLoops(ILooper enabledLooper) {
		// No loops to enable
    }
    
    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();

        if (wantDeploy_ != isDeployed_) {
            if (wantDeploy_) {
                flipper_.set(DEPLOYED_VALUE);
            } else {
                if (super.io_.reverseLimit) { //Don't stow hood until hood is retracted
                    flipper_.set(STOWED_VALUE);
                }
            }
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();

        isDeployed_ = flipper_.get() == DEPLOYED_VALUE;
    }

	@Override
	public void outputTelemetry() {
		// Nothing to output for now		
	}

	@Override
	protected boolean atReverseLimit() {
        return !hasBeenZeroed;
	}

	@Override
	protected boolean atForwardLimit() {
		return !hasBeenZeroed;
	}

	@Override
	protected boolean handleZeroing() { 
		return false; // Hood should be zeroed at specific times so this  is not constantly called
	}

    @Override
    public boolean runActiveTests() { // Tests for this will be run in shooter class
        return true;
    }

}
