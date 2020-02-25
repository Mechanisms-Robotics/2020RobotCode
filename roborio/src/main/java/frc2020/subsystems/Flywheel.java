package frc2020.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;

public class Flywheel extends SingleMotorSubsystem {

    private static Flywheel instance_;

    // TODO: Set speed for actual robot
    private static final int FLYWHEEL_SPEED = 5000;
    private static final int LONG_RANGE_SPEED = 6000;

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS = 
        new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 10;
        masterConstants.invertMotor_ = false;

        MotorConstants[] slaveConstantsArray = new MotorConstants[1];
        var slaveConstants = new MotorConstants();
        slaveConstants.id_ = 11;
        slaveConstants.invertMotor_ = true;
        slaveConstantsArray[0] = slaveConstants;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.slaveConstants_ = slaveConstantsArray;
        DEFAULT_CONSTANTS.name_ = "Flywheel";
        DEFAULT_CONSTANTS.velocityDeadBand_ = 50; // rpm
        DEFAULT_CONSTANTS.velocityKp_ = 0.0006;
        DEFAULT_CONSTANTS.velocityKi_ = 0.0;
        DEFAULT_CONSTANTS.velocityKd_ = 0.0;
        DEFAULT_CONSTANTS.velocityKf_ = 0.00019;
        DEFAULT_CONSTANTS.useBreakMode = true;

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

    public synchronized void spinLongRangeFlywheel() {
        super.setVelocity(LONG_RANGE_SPEED);
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
