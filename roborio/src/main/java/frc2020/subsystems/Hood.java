package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends SingleMotorSubsystem {

    private static Hood instance_;

    private final static int FLIPPER_FORWARD_PORT = 2;
    private final static int FLIPPER_REVERSE_PORT = 3;
    private final static DoubleSolenoid.Value STOWED_VALUE = Value.kForward;
    private final static DoubleSolenoid.Value DEPLOYED_VALUE = Value.kReverse;

    /*TODO: when we have the robot, set this value to halfway between all the way back
            and at the forward position of the reverse limit switch
    */
    private final static double STOW_POSITION = 0.19; // encoder units

    private DoubleSolenoid flipper_;
    private boolean wantDeploy_ = false;
    private boolean isDeployed_ = false;
    private final static DriverStation DS = DriverStation.getInstance();

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
        new SingleMotorSubsystemConstants();

    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 8;
        masterConstants.invertMotor_ = true;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.name_ = "Hood";
        DEFAULT_CONSTANTS.enableHardLimits_ = true;
        DEFAULT_CONSTANTS.useBreakMode = true;
        DEFAULT_CONSTANTS.enableSoftLimits = true;

        DEFAULT_CONSTANTS.forwardSoftLimit = 2.99F;
        DEFAULT_CONSTANTS.reverseSoftLimit = 0.20F;
        DEFAULT_CONSTANTS.homePosition_ = 0.0;

        DEFAULT_CONSTANTS.deadband_ = 0.10;

        DEFAULT_CONSTANTS.kP_ = 0.0005;
        DEFAULT_CONSTANTS.kI_ = 0.0;
        DEFAULT_CONSTANTS.kD_ = 0.0;
        DEFAULT_CONSTANTS.kF_ = 0.0010;
        DEFAULT_CONSTANTS.cruiseVelocity_ = 500.0;
        DEFAULT_CONSTANTS.acceleration_ = 500.0;

        DEFAULT_CONSTANTS.minOutput = -0.3;
        DEFAULT_CONSTANTS.maxOutput = 0.3;
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
    public void zeroSensors() {

	}
    
    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();

        if (wantDeploy_ != isDeployed_) {
            if (wantDeploy_) {
                flipper_.set(DEPLOYED_VALUE);
            } else {
                if (super.atPosition(STOW_POSITION)) { //Don't stow hood until hood is retracted
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
        super.outputTelemetry();

		SmartDashboard.putBoolean("Hood Deployed", isDeployed_);		
	}

	@Override
	protected boolean atReverseLimit() {
        return !hasBeenZeroed || !isDeployed_;
	}

	@Override
	protected boolean atForwardLimit() {
        return !hasBeenZeroed || !isDeployed_;
	}

	@Override
	protected boolean handleZeroing() { 
		final boolean enableZeroing = true;
        if (enableZeroing) {
            if (DS.isEnabled()) {
                encoder.setPosition(constants_.homePosition_);
                return true;
            }
        }
        return false;
	}

    @Override
    public boolean runActiveTests() { // Tests for this will be run in shooter class
        return true;
    }

}
