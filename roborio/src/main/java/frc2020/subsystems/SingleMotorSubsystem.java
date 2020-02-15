package frc2020.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Timer;
import frc2020.util.Logger;
import frc2020.util.Util;

/**
 * Abstract class for a subsystem with a single sensor and one degree
 * of freedom
 */
public abstract class SingleMotorSubsystem implements Subsystem {
    private static final int MOTION_PROFILE_SLOT = 0;
    private static final int POSITION_PID_SLOT = 1;
    private static final int VELOCITY_PID_SLOT = 2;

    /**
     * Basic motor configuration
     */
    public static class MotorConstants {
        public int id_ = -1;
        public boolean invertMotor_ = false;
        public boolean invertSensorPhase_ = false;
    }

    /**
     * Subsystem configuration. Note that all default values here
     * are "safe" meaning that it's fine not to configure them
     * if you don't need them.
     */
    public static class SingleMotorSubsystemConstants {
        public String name_ = "ERROR_NO_NAME";

        public MotorConstants masterConstants_ = new MotorConstants();
        public MotorConstants[] slaveConstants_ = new MotorConstants[0];

        public double homePosition_ = 0.0; // units
        public double positionConversionFactor_ = 1.0;
        public double velocityConversionFactor_ = 1.0;

        public double kP_ = 0.0; // Raw output / raw error
        public double kI_ = 0.0; // Raw output / sum of raw error
        public double kD_ = 0.0; // Raw output / (err - prevErr)
        public int iZone_ = 0;
        public double deadband_ = 0;

        public int cruiseVelocity_ = 0;
        public int acceleration_ = 0;
        public double closedLoopRampRate_ = 0.0;
        public double openLoopRampRate_ = 0.0;
        public int currentLimitStall_ = 30; // Amps
        public int currentLimitFree_ = 30; // Amps
        public boolean useVoltageComp_ = false; // Ask alex before enabling this plz
        public double maxVoltage_ = 12.0;

        public double positionKp_ = 0.0;
        public double positionKi_ = 0.0;
        public double positionKd_ = 0.0;
        public int positionIZone_ = 0;

        public double velocityKp_ = 0.0;
        public double velocityKi_ = 0.0;
        public double velocityKd_ = 0.0;
        public int velocityIZone_ = 0;
        public double velocityDeadBand_ = 0.0;

        public double maxUnitsLimit_ = Double.POSITIVE_INFINITY;
        public double minUnitsLimit_ = Double.NEGATIVE_INFINITY;

        public boolean enableHardLimits_ = true;
    }
    
    protected final SingleMotorSubsystemConstants constants_;
    protected final CANSparkMax sparkMaster_;
    protected final CANPIDController masterPid_;
    protected final CANSparkMax[] sparkSlaves_;

    protected final CANDigitalInput forwardSwitch_;
    protected final CANDigitalInput reverseSwitch_;

    protected CANEncoder encoder;

    protected final int forwardSoftLimitTicks_;
    protected final int reverseSoftLimitTicks_;

    protected Logger logger_ = Logger.getInstance();
    protected final String logName_;

    // TODO: Add error handling where needed
    protected SingleMotorSubsystem(final SingleMotorSubsystemConstants constants) {
        constants_ = constants;
        forwardSoftLimitTicks_ = (int)(constants_.maxUnitsLimit_ - constants_.homePosition_);
        reverseSoftLimitTicks_ = (int)(constants_.minUnitsLimit_ - constants_.homePosition_);

        sparkMaster_ = new CANSparkMax(constants_.masterConstants_.id_, MotorType.kBrushless);
        sparkMaster_.restoreFactoryDefaults();
        sparkSlaves_ = new CANSparkMax[constants_.slaveConstants_.length];

        sparkMaster_.setInverted(constants_.masterConstants_.invertMotor_);
        sparkMaster_.setOpenLoopRampRate(constants_.openLoopRampRate_);
        sparkMaster_.setClosedLoopRampRate(constants.closedLoopRampRate_);
        sparkMaster_.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        sparkMaster_.setSmartCurrentLimit(constants_.currentLimitStall_, constants_.currentLimitFree_);

        if (constants_.useVoltageComp_) {
            sparkMaster_.enableVoltageCompensation(constants_.maxVoltage_);
        } else {
            sparkMaster_.disableVoltageCompensation();
        }

        masterPid_ = sparkMaster_.getPIDController();

        encoder = sparkMaster_.getEncoder();
        encoder.setPositionConversionFactor(constants_.positionConversionFactor_);
        encoder.setVelocityConversionFactor(constants_.velocityConversionFactor_);
        encoder.setInverted(constants_.masterConstants_.invertSensorPhase_);
        masterPid_.setFeedbackDevice(sparkMaster_.getEncoder());

        forwardSwitch_ = sparkMaster_.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
        forwardSwitch_.enableLimitSwitch(constants_.enableHardLimits_);
        reverseSwitch_ = sparkMaster_.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
        reverseSwitch_.enableLimitSwitch(constants_.enableHardLimits_);

        masterPid_.setP(constants_.velocityKp_, VELOCITY_PID_SLOT);
        masterPid_.setI(constants_.velocityKi_, VELOCITY_PID_SLOT);
        masterPid_.setD(constants_.velocityKd_, VELOCITY_PID_SLOT);
        masterPid_.setIZone(constants_.velocityIZone_, VELOCITY_PID_SLOT);
        masterPid_.setFF(0.0, VELOCITY_PID_SLOT);

        masterPid_.setP(constants_.positionKp_, POSITION_PID_SLOT);
        masterPid_.setI(constants_.positionKi_, POSITION_PID_SLOT);
        masterPid_.setD(constants_.positionKd_, POSITION_PID_SLOT);
        masterPid_.setIZone(constants_.positionIZone_, POSITION_PID_SLOT);
        masterPid_.setFF(0.0, POSITION_PID_SLOT);

        masterPid_.setP(constants_.kP_, MOTION_PROFILE_SLOT);
        masterPid_.setI(constants_.kI_, MOTION_PROFILE_SLOT);
        masterPid_.setD(constants_.kD_, MOTION_PROFILE_SLOT);
        masterPid_.setIZone(constants_.iZone_, MOTION_PROFILE_SLOT);
        masterPid_.setFF(0.0, MOTION_PROFILE_SLOT);
        masterPid_.setSmartMotionAllowedClosedLoopError(constants_.deadband_, MOTION_PROFILE_SLOT);
        masterPid_.setSmartMotionMaxAccel(constants_.acceleration_, MOTION_PROFILE_SLOT);
        masterPid_.setSmartMotionMaxVelocity(constants_.cruiseVelocity_, MOTION_PROFILE_SLOT);


        for (int i = 0; i < sparkSlaves_.length; ++i) {
            sparkSlaves_[i].restoreFactoryDefaults();
            sparkSlaves_[i] = new CANSparkMax(constants_.slaveConstants_[i].id_, MotorType.kBrushless);
            sparkSlaves_[i].restoreFactoryDefaults();
            sparkSlaves_[i].follow(sparkMaster_);
            sparkSlaves_[i].setInverted(constants_.slaveConstants_[i].invertMotor_);
        }

        logName_ = constants_.name_;
    }

    public static class PeriodicIO {
        // Inputs
        public double timestamp;
        public double position;
        public double velocity;
        public double outputPercent;
        public double outputVoltage;
        public double masterCurrent;
        public boolean forwardLimit;
        public boolean reverseLimit;

        // Outputs
        public double feedforward;
        public double demand;
    }

    protected enum ControlState {
        OPEN_LOOP, MOTION_PROFILING, POSITION_PID, VELOCITY_PID
    }

    protected PeriodicIO io_ = new PeriodicIO();
    protected ControlState state_ = ControlState.OPEN_LOOP;
    protected boolean hasBeenZeroed = false;

    @Override
    public synchronized void readPeriodicInputs() {
        io_.timestamp = Timer.getFPGATimestamp();
        io_.masterCurrent = sparkMaster_.getOutputCurrent();
        io_.outputPercent = sparkMaster_.getAppliedOutput();
        io_.outputVoltage = io_.outputPercent * sparkMaster_.getBusVoltage();
        io_.position = encoder.getPosition();
        io_.velocity = encoder.getVelocity();
        io_.forwardLimit = forwardSwitch_.get();
        io_.reverseLimit = reverseSwitch_.get();
    }

    // TODO: Add error checking on CAN Bus calls
    @Override
    public synchronized void writePeriodicOutputs() {
        if (!hasBeenZeroed) {
            hasBeenZeroed = handleZeroing();
        }
        if (io_.demand > 0 && atForwardLimit()) {
            return;
        }
        if (io_.demand < 0 && atReverseLimit()) {
            return;
        }
        if (state_ == ControlState.MOTION_PROFILING) {
            masterPid_.setReference(io_.demand, ControlType.kSmartMotion, MOTION_PROFILE_SLOT, io_.feedforward);
        } else if (state_ == ControlState.POSITION_PID) {
            masterPid_.setReference(io_.demand, ControlType.kPosition, POSITION_PID_SLOT, io_.feedforward);
        } else if (state_ == ControlState.VELOCITY_PID) {
            masterPid_.setReference(io_.demand, ControlType.kVelocity, VELOCITY_PID_SLOT, io_.feedforward);
        } else {
            masterPid_.setReference(io_.demand, ControlType.kDutyCycle);
        }
    }

    @Override
    public synchronized boolean runPassiveTests(){
        //TODO: Add temp checks
        boolean hasPassedTests = true;
        return hasPassedTests;
    }

    public synchronized double getPosition() {
        return io_.position;
    }

    public synchronized double getVelocity() {
        return io_.velocity;
    }

    public synchronized void setSmartPosition(double units) {
        setSmartPosition(units, 0.0);
    }
    public synchronized void setSmartPosition(double units, double feedforward) {
        io_.demand = units;
        io_.feedforward = feedforward;
        if (state_ != ControlState.MOTION_PROFILING) {
            state_ = ControlState.MOTION_PROFILING;
        }
    }

    public synchronized void setPosition(double units) {
        setPosition(units, 0.0);
    }

    public synchronized void setPosition(double units, double feedforward) {
        io_.demand = units;
        io_.feedforward = feedforward;
        if (state_ != ControlState.POSITION_PID) {
            state_ = ControlState.POSITION_PID;
        }
    }
    public synchronized void setVelocity(double units) {
        setVelocity(units, 0.0);
    }

    public synchronized void setVelocity(double units, double feedforward) {
        io_.demand = units;
        io_.feedforward = feedforward;
        if (state_ != ControlState.VELOCITY_PID) {
            state_ = ControlState.VELOCITY_PID;
        }
    }

    public synchronized void setOpenLoop(double units) {
        setOpenLoop(units, 0.0);
    }

    public synchronized void setOpenLoop(double units, double feedforward) {
        io_.demand = units;
        io_.feedforward = feedforward;
        if (state_ != ControlState.POSITION_PID) {
            state_ = ControlState.POSITION_PID;
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
    }

    public synchronized boolean atDemand() {
        switch (state_){
            case POSITION_PID:
            case MOTION_PROFILING:
                return Util.epsilonEquals(io_.demand, getPosition(), constants_.deadband_);
            case VELOCITY_PID:
                return Util.epsilonEquals(io_.demand, getVelocity(), constants_.velocityDeadBand_);
            default:
                return true;
        }
    }

    /**
     * Get whether or not the reverse software limit is met
     * This is not about limit switches.
     * This is about programatic conditions where subclasses indicate motion should stop.
     * @return True if the limit condition is met otherwise false
     */
    protected abstract boolean atReverseLimit();

    /**
     * Get whether or not the forward limit condition is met    
     * This is not about limit switches.
     * This is about programatic conditions where subclasses indicate motion should stop.
     * @return True if limit condition is met otherwise false
     */
    protected abstract boolean atForwardLimit();

    /**
     * Used to define the zeroing condition of the subsystem.
     * Note that if you have no zeroing condition just return true
     * @use <p>If the zero is where the system starts just return true</p>
     *      <p>If you have a subsystem that zeros when it hits a limit switch
     *         set the sensor to the known position of the limit switch
     *         and then return true</p>
     * @return True if zeroing occurred false otherwise.
     */
    protected abstract boolean handleZeroing();
}
