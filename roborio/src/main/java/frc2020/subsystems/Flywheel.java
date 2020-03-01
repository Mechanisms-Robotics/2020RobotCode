package frc2020.subsystems;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;

import java.util.concurrent.ConcurrentNavigableMap;

public class Flywheel extends SingleMotorSubsystem {

    private static Flywheel instance_;

    // TODO: Set speed for actual robot
    private static final int FLYWHEEL_SPEED = Constants.IS_COMP_BOT ? 5600 : 5000;
    private static final int LONG_RANGE_SPEED = 6000;

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS = 
        new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 10;
        masterConstants.invertMotor_ = Constants.IS_COMP_BOT ? true : false;

        MotorConstants[] slaveConstantsArray = new MotorConstants[1];
        var slaveConstants = new MotorConstants();
        slaveConstants.id_ = 11;
        slaveConstants.invertMotor_ = true;
        slaveConstantsArray[0] = slaveConstants;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.slaveConstants_ = slaveConstantsArray;
        DEFAULT_CONSTANTS.name_ = "Flywheel";
        DEFAULT_CONSTANTS.velocityDeadBand_ = 100; // rpm
        DEFAULT_CONSTANTS.velocityKp_ = Constants.IS_COMP_BOT ? 0.1583: 0.0006;
        DEFAULT_CONSTANTS.velocityKi_ = 0.0;
        DEFAULT_CONSTANTS.velocityKd_ = 0.0;
        DEFAULT_CONSTANTS.velocityKf_ = Constants.IS_COMP_BOT ? 0.0 : 0.00019;
        DEFAULT_CONSTANTS.useBreakMode = true;

    }

    private SimpleMotorFeedforward feedforward_;
    private static final double KS = 0.0497;
    private static final double KV = 0.13;
    private static final double KA = 0.0357;

    protected Flywheel(SingleMotorSubsystemConstants constants) {
        super(constants);
        feedforward_ = new SimpleMotorFeedforward(KS, KV, KA);
    }

    public static Flywheel getInstance() {
        return instance_ == null ? instance_ = new Flywheel(DEFAULT_CONSTANTS) : instance_;
    }

    @Override
    public synchronized void setVelocity(double units) {
        if (Constants.IS_COMP_BOT) {
            double feedforward = feedforward_.calculate(units);
            super.setVelocity(units, feedforward);
        } else {
            super.setVelocity(units);
        }

    }

    /**
     * Spins flywheel at set speed
     */
    public synchronized void spinFlywheel() {
        this.setVelocity(FLYWHEEL_SPEED);
    }

    public synchronized void spinLongRangeFlywheel() {
        this.setVelocity(LONG_RANGE_SPEED);
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
        // We never want the flywheel to run backwards
        return true;
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
