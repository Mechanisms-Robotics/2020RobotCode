package frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc2020.util.drivers.PhoenixFactory;
import frc2020.util.drivers.TalonSRXUtil;
import frc2020.util.geometry.Rotation2d;
import frc2020.util.DriveSignal;
import frc2020.util.ReflectingCSVWriter;
import frc2020.loops.Loop;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The base drive train class for the 2019 robot. It contains all functions
 * required to move for both autonomous and telop periods of the game.
 * Path following logic inspired by 254
 *
 * @author Team 4910
 */
public class Drive implements Subsystem {

    private static final double DRIVE_ENCODER_PPR = 4096.0;
    private static final int VELOCITY_SLOT = 0;

    // This is the instance_ of the robot that is running on the Robot
    public static Drive instance_;

    /**
     * Contains all the possible states for the drive train
     *
     * @author Team 4910
     */
    public enum DriveState {
        OpenLoop,
        Velocity
    }

    /**
     * The ramp modes supported by DriveTain
     */
    public enum RampMode {
        None,
        LowLift,
        HighLift
    }

    /**
     * The instance_ of DriveTrain
     *
     * @return The current instance_ of DriveTrain
     */
    public static Drive getInstance() {
        if (instance_ == null) {
            instance_ = new Drive();
        }
        return instance_;
    }

    // State
    private DriveState state_;

    // Motors
    private TalonSRX leftMaster_, rightMaster_;
    private TalonSRX leftSlave_, rightSlave_;

    // Shifterd
    private DoubleSolenoid shifter_;
    private DoubleSolenoid.Value lowGear_ = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value highGear_ = DoubleSolenoid.Value.kReverse;

    // IO
    // This is to prevent overwhelming the CAN Bus / HAL
    private PeriodicIO io_;

    private ReflectingCSVWriter<PeriodicIO> CSVWriter_ = null;

    // Inversion
    private boolean invertLeft_ = true;
    private boolean invertRight_ = false;


    /**
     * The default constructor starts the drive train and sets it up to be in
     * OpenLoop mode
     */
    private Drive() {
        io_ = new PeriodicIO();

        // Configure Left Master CANTalon
        leftMaster_ = PhoenixFactory.createDefaultTalon(Constants.LEFT_MASTER_PORT);
        TalonSRXUtil.checkError(
                leftMaster_.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
                        Constants.STATUS_FRAME_TIME,
                        Constants.CAN_TIMEOUT),
                "configure left master feedback frame period");
        leftMaster_.set(ControlMode.PercentOutput, 0);

        TalonSRXUtil.checkError(
                leftMaster_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                        0,
                        Constants.CAN_TIMEOUT),
                "configure left master encoder");
        leftMaster_.setInverted(invertLeft_);
        leftMaster_.setSensorPhase(false);

        // Configure Right Master CANTalon
        rightMaster_ = PhoenixFactory.createDefaultTalon(Constants.RIGHT_MASTER_PORT);
        TalonSRXUtil.checkError(
                rightMaster_.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
                    Constants.STATUS_FRAME_TIME,
                    Constants.CAN_TIMEOUT),
                "configure right master feedback frame period");
        rightMaster_.set(ControlMode.PercentOutput, 0);

        TalonSRXUtil.checkError(
                rightMaster_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                        0,
                        Constants.CAN_TIMEOUT),
                "configure right master feedback sensor");
        rightMaster_.setInverted(invertRight_);
        rightMaster_.setSensorPhase(false);

        // Configure Left Slave CANTalon
        leftSlave_ = PhoenixFactory.createPermanentSlaveTalon(
                Constants.LEFT_SLAVE_PORT,
                leftMaster_);
        leftSlave_.setInverted(InvertType.FollowMaster);

        // Configure Right Slave CANTalon
        rightSlave_ = PhoenixFactory.createPermanentSlaveTalon(
                Constants.RIGHT_SLAVE_PORT,
                rightMaster_);
        rightSlave_.setInverted(InvertType.FollowMaster);

        // Configure Shifters
        shifter_ = new DoubleSolenoid(Constants.SHIFT_FORWARD, Constants.SHIFT_REVERSE);
        state_ = DriveState.OpenLoop;

        setBrakeMode(true);

        loadGains();
    }

    /**
     * Handles the periodic function for the drive train
     */
    private final Loop DriveLoop = new Loop() {

        /**
         * Handles any enabling tasks for the drive train.
         */
        public void init() {
            synchronized (Drive.this) {
                openLoop(new DriveSignal(0, 0));
                setBrakeMode(true);
            }
        }

        /**
         * The function that handles any periodic tasks that the drive needs.
         */
        public void run() {
            synchronized (Drive.this) {
                switch (state_) {
                    case OpenLoop:
                        break;
                    case Velocity:
                        break;
                    default:
                        System.out.println("Invalid drive state_: " + state_);
                }
            }
        }

        /**
         * Handles any tasks for the drive train on disableing
         */
        public void end() {
            openLoop(new DriveSignal(0, 0));
        }
    };

    /**
     * Returns the Iterate Drive Loop
     *
     * @return The Drive Loop
     */
    public Loop getLoop() {
        return DriveLoop;
    }

    @Override
    public void registerLoops(ILooper in){
        in.registure(DriveLoop);
    }

    /**
     * Set all the CANTalons breakMode
     *
     * @param mode the mode tio set the break on the CANTalons to
     */
    public synchronized void setBrakeMode(boolean mode) {
        if (mode) {
            rightMaster_.setNeutralMode(NeutralMode.Brake);
            leftMaster_.setNeutralMode(NeutralMode.Brake);
            rightSlave_.setNeutralMode(NeutralMode.Brake);
            leftSlave_.setNeutralMode(NeutralMode.Brake);
        } else {
            rightMaster_.setNeutralMode(NeutralMode.Coast);
            leftMaster_.setNeutralMode(NeutralMode.Coast);
            rightSlave_.setNeutralMode(NeutralMode.Coast);
            leftSlave_.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void stop(){
        openLoop(DriveSignal.NEUTRAL);
    }

    /**
     * Drive the robot off of voltage.
     * going off position and use power
     *
     * @param signal
     */
    public synchronized void openLoop(DriveSignal signal) {
        if (state_ != DriveState.OpenLoop) {
            leftMaster_.configNominalOutputForward(0, Constants.CAN_TIMEOUT);
            rightMaster_.configNominalOutputForward(0, Constants.CAN_TIMEOUT);

            leftMaster_.configNominalOutputReverse(0, Constants.CAN_TIMEOUT);
            rightMaster_.configNominalOutputReverse(0, Constants.CAN_TIMEOUT);
            state_ = DriveState.OpenLoop;
        }
        setBrakeMode(signal.getBrakeMode());
        io_.left_demand = signal.getLeft();
        io_.right_demand = signal.getRight();
        io_.left_feedforward = 0.0;
        io_.right_feedforward = 0.0;
    }

    /**
     * Drive the robot at a velocity this also sets it to path following mode.
     *
     * @param signal      The velocities to drive at
     * @param feedforward The calculated feedforward values to use
     */
    public synchronized void driveVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (state_ != DriveState.Velocity) {
            leftMaster_.selectProfileSlot(VELOCITY_SLOT, 0);
            rightMaster_.selectProfileSlot(VELOCITY_SLOT, 0);

            state_ = DriveState.Velocity;
        }
        setBrakeMode(signal.getBrakeMode());
        io_.left_demand = signal.getLeft();
        io_.right_demand = signal.getRight();
        io_.left_feedforward = feedforward.getLeft();
        io_.right_feedforward = feedforward.getRight();
    }

    /**
     * Drive the robot at a given velocity
     * @param signal The velocity to drive
     */
    public synchronized void driveVelocity(DriveSignal signal) {
        if (state_ != DriveState.Velocity) {
            leftMaster_.selectProfileSlot(VELOCITY_SLOT, 0);
            rightMaster_.selectProfileSlot(VELOCITY_SLOT, 0);

            state_ = DriveState.Velocity;
        }
        setBrakeMode(signal.getBrakeMode());
        io_.left_demand = signal.getLeft();
        io_.right_demand = signal.getRight();
        io_.left_feedforward = Constants.DRIVE_KV;
        io_.right_feedforward = Constants.DRIVE_KV;
    }

    @Override
	public synchronized void zeroSensors(){
		leftMaster_.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		rightMaster_.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
	}


    /**
     * Loads the PID gains onto the TalonSRXs
     */
    private synchronized void loadGains() {
        leftMaster_.config_kP(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KP, Constants.CAN_TIMEOUT);
        leftMaster_.config_kD(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KD, Constants.CAN_TIMEOUT);
        leftMaster_.config_kI(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KI, Constants.CAN_TIMEOUT);
        leftMaster_.config_kF(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KF, Constants.CAN_TIMEOUT);
        leftMaster_.config_IntegralZone(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_I_ZONE, Constants.CAN_TIMEOUT);


        rightMaster_.config_kP(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KP, Constants.CAN_TIMEOUT);
        rightMaster_.config_kD(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KD, Constants.CAN_TIMEOUT);
        rightMaster_.config_kI(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KI, Constants.CAN_TIMEOUT);
        rightMaster_.config_kF(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_KF, Constants.CAN_TIMEOUT);
        rightMaster_.config_IntegralZone(VELOCITY_SLOT, Constants.VELOCITY_HIGH_GEAR_I_ZONE, Constants.CAN_TIMEOUT);
    }

    /**
     * Returns the state_ of the drive train
     *
     * @return The current dive state_.
     */
    public DriveState getState() {
        return state_;
    }

    /**
     * Cycles the gearing.
     */
    public synchronized void shift() {
        if (shifter_.get() == highGear_) {
            shifter_.set(lowGear_);
        } else {
            shifter_.set(highGear_);
        }
    }

    /**
     * Sets drive to high gear
     */
    public synchronized void setHighGear() {
        shifter_.set(highGear_);
    }

    public synchronized void setHighGear(boolean high_gear) {
        shifter_.set(high_gear ? highGear_ : lowGear_);
    }

    /**
     * Sets drive to low gear
     */
    public synchronized void setLowGear() {
        shifter_.set(lowGear_);
    }

    /**
     * Sets Ramp Mode
     *
     * @param ramp the RampMode to go to
     */
    public synchronized void setRampMode(RampMode ramp) {
        switch (ramp) {
            case None:
                leftMaster_.configOpenloopRamp(0, Constants.CAN_TIMEOUT);
                rightMaster_.configOpenloopRamp(0, Constants.CAN_TIMEOUT);
                break;
            case LowLift:
                leftMaster_.configOpenloopRamp(0.25, Constants.CAN_TIMEOUT);
                rightMaster_.configOpenloopRamp(0.25, Constants.CAN_TIMEOUT);
                break;
            case HighLift:
                leftMaster_.configOpenloopRamp(1, Constants.CAN_TIMEOUT);
                rightMaster_.configOpenloopRamp(1, Constants.CAN_TIMEOUT);
                break;
            default:
                System.out.println("ERROR Invalad ramp mode receved.");
                return;
        }
        System.out.println("[INFO] Ramp mode set to " + ramp.toString());
    }

    public synchronized boolean getGear() {
        if (shifter_.get() == lowGear_) {
            return true;
        } else {
            return false;
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.WHEEL_DIAMETER * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.WHEEL_DIAMETER * Math.PI);
    }

    private static double rotationsToEncoderTicks(double rotations) {
        return rotations * DRIVE_ENCODER_PPR;
    }

    private static double inchesToEncoderTicks(double inches) {
        return rotationsToEncoderTicks(inchesToRotations(inches));
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    public double getLeftEncoderRotations() {
        return io_.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return io_.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return io_.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return io_.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.DRIVE_TRACK_WIDTH;
    }

    public double getAvgVoltage() {
        return (io_.left_voltage+io_.right_volatge)/2;
    }

    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = io_.left_position_ticks;
        double prevRightTicks = io_.right_position_ticks;
        io_.left_position_ticks = leftMaster_.getSelectedSensorPosition(0);
        io_.right_position_ticks = rightMaster_.getSelectedSensorPosition(0);
        io_.left_velocity_ticks_per_100ms = leftMaster_.getSelectedSensorVelocity(0);
        io_.right_velocity_ticks_per_100ms = rightMaster_.getSelectedSensorVelocity(0);

        double deltaLeftTicks = ((io_.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            io_.left_distance += deltaLeftTicks * Constants.WHEEL_DIAMETER;
        } else {
            io_.left_distance += deltaLeftTicks * Constants.WHEEL_DIAMETER;
        }

        double deltaRightTicks = ((io_.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0) {
            io_.right_distance += deltaRightTicks * Constants.WHEEL_DIAMETER;
        } else {
            io_.right_distance += deltaRightTicks * Constants.WHEEL_DIAMETER;
        }

        io_.right_volatge = rightMaster_.getMotorOutputVoltage();
        io_.left_voltage = leftMaster_.getMotorOutputVoltage();

        if (CSVWriter_ != null) {
            CSVWriter_.add(io_);
        }
    }

    public synchronized void writePeriodicOutputs() {
        if (state_ == DriveState.OpenLoop) {
            leftMaster_.set(ControlMode.PercentOutput, io_.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            rightMaster_.set(ControlMode.PercentOutput, io_.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        } else {
            leftMaster_.set(ControlMode.Velocity, io_.left_demand, DemandType.ArbitraryFeedForward,
                    io_.left_feedforward + Constants.VELOCITY_HIGH_GEAR_KD * io_.left_accel / 1023.0);
            rightMaster_.set(ControlMode.Velocity, io_.right_demand, DemandType.ArbitraryFeedForward,
                    io_.right_feedforward + Constants.VELOCITY_HIGH_GEAR_KD * io_.right_accel / 1023.0);
        }
    }

    public static class PeriodicIO {
        // INPUTS
        int left_position_ticks;
        int right_position_ticks;
        double left_distance;
        double right_distance;
        int left_velocity_ticks_per_100ms;
        int right_velocity_ticks_per_100ms;
        Rotation2d gyro_heading = Rotation2d.identity();
        double left_voltage;
        double right_volatge;


        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
    }

    @Override
    public void writeToLog() {

    }

    public synchronized void startLogging() {
        if (CSVWriter_ == null) {
            CSVWriter_ = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (CSVWriter_ != null) {
            CSVWriter_.flush();
            CSVWriter_ = null;
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", io_.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", io_.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", io_.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", io_.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        if (CSVWriter_ != null) {
            CSVWriter_.write();
        }
    }

    @Override
    public boolean checkSystem(){
        System.out.println("[ERROR] System Check Not implemented");
        return false;
    }
}