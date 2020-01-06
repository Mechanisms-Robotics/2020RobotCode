package frc2020.subsystems;

import frc2020.util.drivers.NavX;

import frc2020.util.DriveSignal;
import frc2020.util.ReflectingCSVWriter;
import frc2020.loops.Loop;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The base drive train class for the 2019 robot. It contains all functions
 * required to move for both autonomous and teleop periods of the game.
 * Path following logic inspired by 254
 *
 * @author Team 4910
 */
public class Drive implements Subsystem {

    private static final double DRIVE_ENCODER_PPR = 4096.0;
    private static final double NEO_ENCODER_PPR = 46.0;
    private static final double HIGH_GEAR_RATIO = 8.96; 
    
    private static final int VELOCITY_SLOT = 0;

    // This is the instance_ of the Drive object on the robot
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
     * The instance_ of Drive
     *
     * @return The current instance_ of Drive
     */
    public static Drive getInstance() {
        if (instance_ == null) {
            instance_ = new Drive();
        }
        return instance_;
    }

    // State
    private DriveState state_;

    //NavX
    private NavX gyro_;

    // Motors
    // TODO: SPARK MAX Definitions

    //Encoders
    // TODO: Implement CANCoders

    // Shifter
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
        // TODO: Define and configure spark max contorlers
        // See if there is a way to check there errors like
        // We did with the Talons

        io_ = new PeriodicIO();
        // Configure NavX
        gyro_ = new NavX(SerialPort.Port.kUSB);
        setBrakeMode(true);
        loadGains();
    }

    /**
     * Handles the periodic function for the drive train
     */
    private final Loop DriveLoop = new Loop() {

        /**
         * Handles any intialization tasks for the drive train
         */
        public void init() {
            synchronized (Drive.this) {
                openLoop(new DriveSignal(0, 0));
                setBrakeMode(true);
            }
        }

        /**
         * Handles any periodic tasks for the drive train
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
         * Handles any tasks for the drive train on disabling
         */
        public void end() {
            openLoop(new DriveSignal(0, 0));
        }
    };

    /**
     * Returns the Drive Loop
     *
     * @return The Drive Loop
     */
    public Loop getLoop() {
        return DriveLoop;
    }

    /**
    * Registers the Drive Loop
    */
    @Override
    public void registerLoops(ILooper in){
        in.register(DriveLoop);
    }

    /**
     * Set all the CANTalons to Brake Mode
     *
     * @param mode the mode to set the brake on the CANTalons to
     */
    public synchronized void setBrakeMode(boolean mode) {
        // ADD Brake mode cofiction for SPARK MAX
        if (mode) {
            return;
        } else {
            return;
        }
    }

    /**
    * Sets all the CANTalons' power to 0
    */
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
            // TODO Config Nominal Ouptus of Spark Maxes
            state_ = DriveState.OpenLoop;
        }
        setBrakeMode(signal.getBrakeMode());
        io_.left_demand = signal.getLeft();
        io_.right_demand = signal.getRight();
        io_.left_feedforward = 0.0;
        io_.right_feedforward = 0.0;
    }

    /**
     * Drive the robot at a specified velocity this also sets it to path following mode.
     *
     * @param signal      The velocities to drive at in meters per sec
     * @param feedforward The calculated feedforward values to use
     */
    public synchronized void driveVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (state_ != DriveState.Velocity) {
            // TODO Select correct PID Proflie on Spark Max

            state_ = DriveState.Velocity;
        }
        setBrakeMode(signal.getBrakeMode());
        io_.left_demand = metersPerSecondToRpm(signal.getLeft()) * HIGH_GEAR_RATIO;
        io_.right_demand = metersPerSecondToRpm(signal.getRight()) * HIGH_GEAR_RATIO;
        io_.left_feedforward = feedforward.getLeft();
        io_.right_feedforward = feedforward.getRight();
    }

    /**
     * Drive the robot at a specified velocity this also sets it to path following mode.
     *
     * @param signal The velocities to drive at
     */
    public synchronized void driveVelocity(DriveSignal signal) {
        if (state_ != DriveState.Velocity) {
            // TOTO Select correct PID Proflie on Spark Max

            state_ = DriveState.Velocity;
        }
        setBrakeMode(signal.getBrakeMode());
        io_.left_demand = metersPerSecondToRpm(signal.getLeft()) * HIGH_GEAR_RATIO;
        io_.right_demand = metersPerSecondToRpm(signal.getRight()) * HIGH_GEAR_RATIO;
        io_.left_feedforward = Constants.DRIVE_KV;
        io_.right_feedforward = Constants.DRIVE_KV;
    }

    /**
    * Resets encoder ticks
    */
    @Override
	public synchronized void zeroSensors(){
		// TODO Resect Neo Encoders and CAN Coders
	}


    /**
     * Loads the PID gains onto the CANTalons
     */
    private synchronized void loadGains() {
        // TODO Config Velocity PID on SparkMax
    }

    /**
     * Returns the state_ of the drive train
     *
     * @return The current drive state_.
     */
    public DriveState getState() {
        return state_;
    }

    /**
     * Toggles the robot between low gear or high gear
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

    /**
     * Sets drive to low gear
     */
    public synchronized void setLowGear() {
        shifter_.set(lowGear_);
    }

    public synchronized Rotation2d getHeading() {
        return io_.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        Rotation2d adjustment = heading.rotateBy(gyro_.getRawRotation().unaryMinus());
        gyro_.setAngleAdjustment(adjustment);
    }

    /**
    * Gets the current mode the shifter is in low gear or high gear
    *
    * @return The current shifter state.
    */
    public synchronized boolean getGear() {
        if (shifter_.get() == lowGear_) {
            return true;
        } else {
            return false;
        }
    }

    /**
    * Converts wheel rotations to linear meters
    *
    * @return Linear meters traveled for x wheel rotations.
    */
    private static double rotationsToMeters(double rotations) {
        return rotations * (Constants.WHEEL_DIAMETER * Math.PI);
    }

    /**
    * Converts RPM to IPS
    *
    * @return The linear meters traveled per second for a wheel moving at x rpm.
    */
    private static double rpmMetersPerSecond(double rpm) {
        return rotationsToMeters(rpm) / 60;
    }

    /**
    * Converts linear meters to wheel rotations
    *
    * @return Wheel rotations for x linear meters traveled.
    */
    private static double metersToRotations(double meters) {
        return meters / (Constants.WHEEL_DIAMETER * Math.PI);
    }

    /**
    * Converts wheel rotations to encoder ticks
    *
    * @return Encoder ticks for x wheel rotations.
    */
    private static double rotationsToEncoderTicks(double rotations) {
        return rotations * DRIVE_ENCODER_PPR;
    }

    /**
    * Converts linear meters traveled to encoder ticks
    *
    * @return Encoder ticks for x linear meters traveled.
    */
    public static double metersToEncoderTicks(double meters) {
        return rotationsToEncoderTicks(metersToRotations(meters));
    }

    /**
    * Converts IPS to RPM
    *
    * @return The RPM of a wheel that is traveling x IPS.
    */
    private static double metersPerSecondToRpm(double meters_per_second) {
        return metersToRotations(meters_per_second) * 60;
    }

    /**
    * Converts radians per second to encoder ticks per 100 miliseconds
    *
    * @return Encoder ticks per 100 miliseconds for a wheel that is rotating at x radians per second.
    */
    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * NEO_ENCODER_PPR / 10.0;
    }

    /**
    * Gets the left encoder rotations
    *
    * @return Left encoder rotations.
    */
    public double getLeftEncoderRotations() {
        return io_.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    /**
    * Gets the right encoder rotations
    *
    * @return Right encoder rotations.
    */
    public double getRightEncoderRotations() {
        return io_.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    /**
    * Gets the left distance traveled in meters
    *
    * @return Left distance in meters.
    */
    public double getLeftEncoderDistance() {
        return rotationsToMeters(getLeftEncoderRotations());
    }

    /**
    * Gets the right distance traveled in meters
    *
    * @return Right distance in meters.
    */
    public double getRightEncoderDistance() {
        return rotationsToMeters(getRightEncoderRotations());
    }

    /**
    * Gets the right velocity in native units
    *
    * @return Right velocity in native units.
    */
    public double getRightVelocityRPM() {
        return io_.right_velocity_rpm;
    }

    /**
    * Gets the right velocity in MPS
    *
    * @return Right velocity in MPS.
    */
    public double getRightLinearVelocity() {
        return rotationsToMeters(getRightVelocityRPM() / 60.0);
    }

    /**
    * Gets the left velocity in native units
    *
    * @return Left velocity in native units.
    */
    public double getLeftVelocityRPM() {
        return io_.left_velocity_rpm;
    }

    /**
    * Gets the left velocity in MPS
    *
    * @return Left velocity in MPS.
    */
    public double getLeftLinearVelocity() {
        return rotationsToMeters(getLeftVelocityRPM() / 60.0);
    }

    /**
    * Gets the total linear velocity of the robot
    *
    * @return Linear velocity of robot.
    */
    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    /**
    * Gets the total angular velocity of the robot
    *
    * @return Angular velocity of robot.
    */
    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.DRIVE_TRACK_WIDTH;
    }

    /**
    * Handles reading all of the data from encoders/CANTalons periodically
    */
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = io_.left_position_ticks;
        double prevRightTicks = io_.right_position_ticks;

        // Get this from the CAN coders
        io_.left_position_ticks = 0;
        io_.right_position_ticks = 0;

        // Get this from the Spark Maxes
        io_.left_velocity_rpm = 0 / HIGH_GEAR_RATIO;
        io_.right_velocity_rpm = 0 / HIGH_GEAR_RATIO;
        io_.gyro_heading = gyro_.getYaw();

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

        if (CSVWriter_ != null) {
            CSVWriter_.add(io_);
        }
    }

    /**
    * Handles writing outputs to CANTalons periodically
    */
    public synchronized void writePeriodicOutputs() {
        if (state_ == DriveState.OpenLoop) {
            // TODO Set output of Spark Maxes in open loop and an 
            // feedforward of 0.0
        } else {
            // TODO Set velocity output of spark maxes and make sure
            // to pass it the feedforward
        }
    }

    public static class PeriodicIO {
        // INPUTS
        int left_position_ticks;
        int right_position_ticks;
        double left_distance;
        double right_distance;
        double left_velocity_rpm;
        double right_velocity_rpm;
        Rotation2d gyro_heading = new Rotation2d();


        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
    }


    /**
    * Starts logging data to a csv
    */
    public synchronized void startLogging() {
        if (CSVWriter_ == null) {
            CSVWriter_ = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    /**
    * Stops logging data to a csv
    */
    public synchronized void stopLogging() {
        if (CSVWriter_ != null) {
            CSVWriter_.flush();
            CSVWriter_ = null;
        }
    }

    /**
    * Outputs telemetry to SmartDashboard and csv
    */
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
