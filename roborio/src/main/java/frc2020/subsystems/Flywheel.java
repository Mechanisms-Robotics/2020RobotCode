package frc2020.subsystems;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;
import frc2020.util.Units;
import frc2020.util.Logger;
import frc2020.util.SmartDashboardUtil;
import frc2020.util.Util;
import frc2020.subsystems.SingleMotorSubsystem.PeriodicIO;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

public class Flywheel implements Subsystem {

    private static Flywheel instance_;


    private static final double PULLEY_RATIO = 1.0 / 1.5;
    private static final int FLYWHEEL_SPEED = Units.rpmToEncTicksPer100Ms( (int) (6000 * PULLEY_RATIO));
    private static final int LONG_RANGE_SPEED = Units.rpmToEncTicksPer100Ms( (int) (6000 * PULLEY_RATIO));

    private static int currentLimitStall_ = 30; // amps
    private static double maxVoltage_ = 11.5;

    private WPI_TalonFX falconMaster_, falconSlave_;

    private PeriodicIO io_ = new PeriodicIO();

    private int masterId_ = 10;
    private boolean masterInvertMotor_ = true;

    private int slaveId_ = 11;
    private boolean slaveInvertMotor_ = true;
    private String name_ = "Flywheel";

    private int velocityDeadBand_ = 200; // rpm
    private double velocityKp_ = Constants.IS_COMP_BOT ? 0.005: 0.0006; //0.1583
    private double velocityKi_ = 0.0;
    private double velocityKd_ = 0.0;
    private double velocityKf_ = Constants.IS_COMP_BOT ? 0.0 : 0.00019;
    private boolean useBrakeMode = true;
    private boolean useVoltageComp_ = true;
    private final int VELOCITY_PID_SLOT = 0;

    private static final double KS = 0.0497;
    private static final double KV = 0.13;
    private static final double KA = 0.0357;

    private static Logger logger_;

    protected Flywheel() {
        falconMaster_ = new WPI_TalonFX(masterId_);
        falconSlave_ = new WPI_TalonFX(slaveId_);

        falconMaster_.configFactoryDefault();
        falconSlave_.configFactoryDefault();

        falconMaster_.setInverted(masterInvertMotor_);
        falconMaster_.configOpenloopRamp(0.0);
        falconMaster_.configClosedloopRamp(0.0);
        falconMaster_.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
        falconMaster_.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimitStall_, 50.0, 0.5));

        falconMaster_.configVoltageCompSaturation(maxVoltage_);
        falconMaster_.enableVoltageCompensation(true);

        falconMaster_.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, VELOCITY_PID_SLOT, 10);

        falconSlave_.follow(falconMaster_);

        falconMaster_.setNeutralMode(NeutralMode.Coast);
        falconSlave_.setNeutralMode(NeutralMode.Coast);

        falconMaster_.config_kP(VELOCITY_PID_SLOT, velocityKp_);
        falconMaster_.config_kI(VELOCITY_PID_SLOT, velocityKi_);
        falconMaster_.config_kD(VELOCITY_PID_SLOT, velocityKd_);
        falconMaster_.config_kF(VELOCITY_PID_SLOT, velocityKf_);

        logger_ = Logger.getInstance();
    }

    public static Flywheel getInstance() {
        return instance_ == null ? instance_ = new Flywheel() : instance_;
    }

    public synchronized void setVelocity(double units) {
        io_.demand = units;
        double feedforward = 0.0;
        falconMaster_.set(TalonFXControlMode.Velocity, io_.demand, DemandType.ArbitraryFeedForward, feedforward);
    }

    /**
     * Spins flywheel at set speed
     */
    public synchronized void spinFlywheel() {
        //setVelocity(FLYWHEEL_SPEED);
        falconMaster_.set(TalonFXControlMode.PercentOutput, 0.75);
    }

    public synchronized void spinFlywheel(double velocity) {
        setVelocity(velocity);
    }

    public synchronized void spinLongRangeFlywheel() {
        falconMaster_.set(TalonFXControlMode.Velocity, LONG_RANGE_SPEED);
    }

    /**
     * @return true if flywheel speed is our desired stable speed
     */
    public synchronized boolean upToSpeed() {
        return Util.epsilonEquals(io_.demand, getVelocity(), velocityDeadBand_);
    }

    public synchronized double getVelocity() {
        return io_.velocity;
    }

    public synchronized void setOpenLoop(double units) {
        //logger_.logDebug("Set open loop units: " + units, logName_);
        io_.demand = units;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        io_.timestamp = Timer.getFPGATimestamp();
        io_.masterCurrent = falconMaster_.getOutputCurrent();
        io_.outputPercent = falconMaster_.getMotorOutputPercent();
        io_.outputVoltage = io_.outputPercent * falconMaster_.getBusVoltage();
        io_.position = falconMaster_.getSelectedSensorPosition();
        io_.velocity = falconMaster_.getSelectedSensorVelocity() / PULLEY_RATIO;
        io_.dutyCycle = falconMaster_.getMotorOutputPercent();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(name_ + " : Position", io_.position);
        SmartDashboard.putNumber(name_ + " : Velocity", Units.encTicksPer100MsToRpm((int) io_.velocity));
        SmartDashboard.putNumber(name_ + " : Demand", Units.encTicksPer100MsToRpm((int) io_.velocity));
        SmartDashboard.putNumber(name_ + " : Duty Cycle", io_.dutyCycle);
        SmartDashboard.putBoolean(name_ + " : Safety is Enabled", falconMaster_.isSafetyEnabled());
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

    public void registerLoops(ILooper enabledLooper) {
        // Nothing to do here
    }

    public void stop() {
        // Nothing to do here
    }

    protected boolean atReverseLimit() {
        // We never want the flywheel to run backwards
        return true;
    }

    protected boolean atForwardLimit() {
        // Not necessary for flywheel
        return false;
    }

    protected boolean handleZeroing() {
        // Nothing to zero
        return true;
    }

    @Override
    public boolean runPassiveTests() {
        return true;
    }

    public synchronized boolean atVelocity(double velocity) {
        return Util.epsilonEquals(velocity, getVelocity(), velocityDeadBand_);
    }

    public synchronized boolean isStopped() {
        return atVelocity(0.0);
    }

    public synchronized boolean atDemand() {
        return Util.epsilonEquals(io_.demand, getVelocity(), velocityDeadBand_);
    }

}