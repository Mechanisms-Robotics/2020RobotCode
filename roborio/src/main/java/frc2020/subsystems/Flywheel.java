package frc2020.subsystems;

import frc2020.loops.ILooper;

public class Flywheel extends SingleMotorSubsystem {

    private static Flywheel instance_;

    // TODO: Set speed for actual robot
    private static int FLYWHEEL_SPEED = 5500;

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS = 
        new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 7;
        masterConstants.invertMotor_ = true;
        masterConstants.invertSensorPhase_ = false;

        var slaveConstants = new MotorConstants();
        slaveConstants.id_ = 8;
        slaveConstants.invertMotor_ = false;
        masterConstants.invertSensorPhase_ = true;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.slaveConstants_ = new MotorConstants[]{slaveConstants};
        DEFAULT_CONSTANTS.name_ = "Flywheel";
        DEFAULT_CONSTANTS.velocityDeadBand_ = 50; // rpm
    }

    protected Flywheel(SingleMotorSubsystemConstants constants) {
        super(constants);
    }

    public static Flywheel getInstance() {
        return instance_ == null ? instance_ = new Flywheel(DEFAULT_CONSTANTS) : instance_;
    }

    /**
     * Spins flywheel at set speed
     */
    public synchronized void spinFlywheel() {
        super.setVelocity(FLYWHEEL_SPEED);
    }

    /**
     * @return true if flywheel speed is our desired stable speed
     */
    public synchronized boolean upToSpeed() {
        return super.atDemand();
    }
    /**
     * Note: do not add active tests here as we would like to run the shooter
     * with the hood component as part of the tests
     */
    @Override
    public boolean runActiveTests() {
        return true;
    }

    @Override
    public void zeroSensors() {
        //No sensors to zero
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        //Not registering any loops
    }

    @Override
    public void outputTelemetry() {
        // No telemetry to output for flywheel
    }

    @Override
    protected boolean atReverseLimit() {
        // Not necessary for flywheel
        return false;
    }

    @Override
    protected boolean atForwardLimit() {
        // Not necessary for flywheel
        return false;
    }

    @Override
    protected boolean handleZeroing() {
        // Nothing to zero
        return true;
    }
    
}