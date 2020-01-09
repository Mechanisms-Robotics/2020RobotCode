package frc2020.subsystems;

import frc2020.util.drivers.NavX;
import frc2020.util.DriveSignal;
import frc2020.util.ReflectingCSVWriter;
import frc2020.util.Logger;
import frc2020.loops.Loop;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

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
    
    // This is the instance_ of the Drive object on the robot
    public static Drive instance_;

    /**
     * Contains all the possible states for the drive train
     *
     * @author Team 4910
     */
    public enum DriveState {
        OpenLoop,
        Velocity,
        Trajectory_Following
    }

    /**
     * The instance of Drive
     *
     * @return The current instance of Drive
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
    private CANCoder leftCanCoder;
    private CANCoder rightCanCoder;

    private CANCoderConfiguration leftCoderConfig;
    private CANCoderConfiguration rightCoderConfig;

    // Shifter
    private DoubleSolenoid shifter_;
    private DoubleSolenoid.Value lowGear_ = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value highGear_ = DoubleSolenoid.Value.kReverse;

    // IO
    // This is to prevent overwhelming the CAN Bus / HAL
    private PeriodicIO io_;

    private ReflectingCSVWriter<PeriodicIO> CSVWriter_ = null;

    // Path Following
    private DifferentialDriveKinematics kinematics_;
    private DifferentialDriveOdometry odometry_;
    private Trajectory currentTrajectory_;
    private double trajectoryStartTime_;
    private RamseteController controller_;
    private SimpleMotorFeedforward feedforward_;
    private boolean doneWithTrajectory_;

    /**
     * The default constructor starts the drive train and sets it up to be in
     * OpenLoop mode
     */
    private Drive() {
        // TODO: Define and configure spark max contorlers
        // See if there is a way to check there errors like
        // We did with the Talons


        io_ = new PeriodicIO();
        configCanCoders();
        // Configure NavX
        gyro_ = new NavX(SerialPort.Port.kUSB);
        setBrakeMode(true);
        loadGains();

        kinematics_ = new DifferentialDriveKinematics(
            Constants.TRACK_SCRUB_FACTOR * Constants.DRIVE_TRACK_WIDTH
        );
        odometry_ = new DifferentialDriveOdometry(new Rotation2d(0.0));
        currentTrajectory_ = null;
        trajectoryStartTime_ = -1.0;
        doneWithTrajectory_ = true;
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
                    case Trajectory_Following:
                        updateTrajectoryFollower();
                        break;
                    case Velocity:
                        break;
                    default:
                        Logger.logWarning("Invalid drive state_: " + state_);
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

    private void updateTrajectoryFollower() {
        if (doneWithTrajectory_) {

            // Check that we know our start time and have a trajectory to follow
            if (trajectoryStartTime_ > 0.0 && currentTrajectory_ != null) {
                
                // Check to see if we are done with the trajectory
                if (trajectoryStartTime_ + currentTrajectory_.getTotalTimeSeconds() > Timer.getFPGATimestamp()) {

                    // Lookup where we are supposed to be on the trajetory
                    double lookup_time = Timer.getFPGATimestamp() - trajectoryStartTime_;
                    Trajectory.State trajectory_state = currentTrajectory_.sample(lookup_time);

                    // Get our adjusted wheel speeds from the ramset controller and send them to the drive train
                    ChassisSpeeds speeds = controller_.calculate(getOdometryPose(), trajectory_state);
                    DifferentialDriveWheelSpeeds drive_speeds = kinematics_.toWheelSpeeds(speeds);
                    driveVelocity(new DriveSignal(drive_speeds, false));
                } else {
                    // TODO: We could write an pose2pose controller that runs if the
                    // robot is too far off from the desired end state of the trajectory
                    
                    // Set the wheels to the desired end velocity of the trajectory
                    double end_speed = currentTrajectory_.sample(
                        currentTrajectory_.getTotalTimeSeconds()).velocityMetersPerSecond;
                    driveVelocity(new DriveSignal(end_speed, end_speed));
                    state_ = DriveState.Velocity;
                    doneWithTrajectory_ = true;
                }
            } else {
                Logger.logError(
                    "Unable to follow trajectory due to invaded start time or trajectory");
                openLoop(new DriveSignal(0, 0, true));
            }
        } else {
            Logger.logWarning(
                "Update path follower called when tarjectory has already been completed"
            );
        }
    }

    /**
    * Registers the Drive Loop
    */
    @Override
    public void registerLoops(ILooper in){
        in.register(DriveLoop);
    }

    /**
     * Set all the Spark Maxes to Brake Mode
     *
     * @param mode the mode to set the brake on the Spark Maxes to
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
    * Sets all the Spark Maxes' power to 0
    */
    @Override
    public void stop(){
        openLoop(DriveSignal.NEUTRAL);
    }

    /**
     * Drive the robot off of voltage.
     * going off position and use power
     *
     * @param signal - the desired DriveSignal
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
        if (state_ != DriveState.Velocity || state_ != DriveState.Trajectory_Following) {
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
        if (state_ != DriveState.Velocity || state_ != DriveState.Trajectory_Following) {
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
     * Drive the robot atomosly on a pre-defined trajectory
     * @param trajectory The trajectory to follow
     */
    public synchronized void driveTrajectory(Trajectory trajectory) {
        if (state_ != DriveState.Velocity || state_ != DriveState.Trajectory_Following) {
            // TODO Set correct PID Profile on the Spark Max

            state_ = DriveState.Trajectory_Following;
        }
        doneWithTrajectory_ = false;
        currentTrajectory_ = trajectory;
        trajectoryStartTime_ = Timer.getFPGATimestamp();
        updateTrajectoryFollower();
    }

    /**
     * @return the doneWithTrajectory_
     */
    public boolean isDoneWithTrajectory() {
        return doneWithTrajectory_;
    }

    /**
    * Resets encoder ticks for CAN Coders and NEO Encoders
    */
    @Override
	public synchronized void zeroSensors(){
        // TODO Reset Neo Encoders
        
        // Sends identity Pose and Rotation2d to resetOdometry function so we're using the same function
        resetOdometry(new Pose2d(), new Rotation2d());
    }
    
    /**
     * Resets the odometry and gyroscope to the desired pose and rotation
     */
    public synchronized void resetOdometry(Pose2d pose, Rotation2d rotation) {
        // Sets odometry to desired pose and rotation
        odometry_.resetPosition(pose, rotation);
        
        // Prints the stack if there are any errors in setting the position of the CAN Coders
        ErrorCode rv = leftCanCoder.setPosition(0, Constants.CAN_TIMEOUT);
        if(rv != ErrorCode.OK) {
            Logger.logError("Left CAN coder reset failed with error: " + rv.toString());
        }

        rv = rightCanCoder.setPosition(0, Constants.CAN_TIMEOUT);
        if(rv != ErrorCode.OK) {
            Logger.logError("Right CAN coder reset failed with error: " + rv.toString());
        }
        
        // Sets the gyroscope to the desired rotation
        setHeading(rotation);
    }


    /**
     * Loads the PID gains onto the Spark Maxes
     */
    private synchronized void loadGains() {
        // TODO Config Velocity PID on Spark Max
    }

    /**
     * Loads the CAN Coder Configuration files to the CAN Coders. Also reports magnet alignment and config errors
     * Encoder polarity, unit coefficients also managed here
     */
    private synchronized void configCanCoders() {
        // Reports firmware version for logging purposes
        Logger.logInfo("Left CAN Coder Firmware: " + leftCanCoder.getFirmwareVersion());
        Logger.logInfo("Right CAN Coder Firmware: " + rightCanCoder.getFirmwareVersion());
        
        // Checks for alignment of magnet with encoder (the encoder light color)
        MagnetFieldStrength leftMagStrength = leftCanCoder.getMagnetFieldStrength();
        if (leftMagStrength == MagnetFieldStrength.BadRange_RedLED) {
            Logger.logError("Left CAN Coder magnet in the red (out of range)");
        } else if (leftMagStrength == MagnetFieldStrength.Adequate_OrangeLED) {
            Logger.logWarning("Left CAN Coder magnet in the orange (slightly out of alignment)");
        } else if (leftMagStrength == MagnetFieldStrength.Good_GreenLED) {
            Logger.logDebug("Left CAN Coder magnet is green (healthy)");
        } else {
            Logger.logError("Left CAN Coder magnet not detected");
        }

        MagnetFieldStrength rightMagStrength = rightCanCoder.getMagnetFieldStrength();
        if (rightMagStrength == MagnetFieldStrength.BadRange_RedLED) {
            Logger.logError("Right CAN Coder magnet in the red (out of range)");
        } else if (rightMagStrength == MagnetFieldStrength.Adequate_OrangeLED) {
            Logger.logWarning("Right CAN Coder magnet in the orange (slightly out of alignment)");
        } else if (rightMagStrength == MagnetFieldStrength.Good_GreenLED) {
            Logger.logDebug("Right CAN Coder magnet is green (healthy)");
        } else {
            Logger.logError("Right CAN Coder magnet not detected");
        }
        
        // CANCoder configuration objects
        leftCoderConfig = new CANCoderConfiguration();
        rightCoderConfig = new CANCoderConfiguration();
        
        // Opposite values because of the orientation of the drives. Switch if the robot is reading distance backwards
        leftCoderConfig.sensorDirection = true;
        rightCoderConfig.sensorDirection = false;

        // This is used to directly convert from encoder units to meters. Note that this coefficient is in m/enc
        double sensorCoefficient = Constants.WHEEL_DIAMETER * Math.PI / DRIVE_ENCODER_PPR;

        leftCoderConfig.sensorCoefficient = sensorCoefficient;
        rightCoderConfig.sensorCoefficient = sensorCoefficient;
        
        // Sets unit name
        leftCoderConfig.unitString = "meters";
        rightCoderConfig.unitString = "meters";

        // Prints the stack if there are any errors in pushing the configuration objects
        ErrorCode rv = leftCanCoder.configAllSettings(leftCoderConfig, Constants.CAN_TIMEOUT);
        if (rv != ErrorCode.OK) {
            Logger.logError("Left CAN coder config failed with error: " + rv.toString());
        }

        rv = rightCanCoder.configAllSettings(rightCoderConfig, Constants.CAN_TIMEOUT);
        if (rv != ErrorCode.OK) {
            Logger.logError("Right CAN coder config failed with error: " + rv.toString());
        }
    }
    /**
     * Returns the state of the drive train
     *
     * @return The current drive state.
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

    /**
     * Gets the gyroscope's current heading
     * 
     * @return Gyro heading as a Rotation2d
     */
    public synchronized Rotation2d getHeading() {
        return io_.gyro_heading;
    }

    /**
     * Sets the gyroscope's heading to a desired Rotation2d
     * 
     * @param heading - the desired gyro heading as a Rotation2d
     */
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
    * Converts RPM to MPS
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
    * Converts MPS to RPM
    *
    * @return The RPM of a wheel that is traveling x MPS.
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
    * Gets the left distance traveled in meters
    *
    * @return Left distance in meters.
    */
    public double getLeftEncoderDistance() {
        return io_.left_distance;
    }

    /**
    * Gets the right distance traveled in meters
    *
    * @return Right distance in meters.
    */
    public double getRightEncoderDistance() {
        return io_.right_distance;
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

    public Pose2d getOdometryPose() {
        return odometry_.getPoseMeters();
    }

    /**
    * Handles reading all of the data from encoders/CANTalons periodically
    */
    public synchronized void readPeriodicInputs() {
        // Get this from the CAN coders
        io_.left_distance = leftCanCoder.getPosition();
        io_.right_distance = rightCanCoder.getPosition();

        // Get this from the Spark Maxes
        io_.left_velocity_rpm = 0 / HIGH_GEAR_RATIO;
        io_.right_velocity_rpm = 0 / HIGH_GEAR_RATIO;
        io_.gyro_heading = gyro_.getYaw();

        odometry_.update(io_.gyro_heading, getLeftEncoderDistance(), getRightEncoderDistance());

        if (CSVWriter_ != null) {
            CSVWriter_.add(io_);
        }
    }

    /**
    * Handles writing outputs to Spark Maxes periodically
    */
    public synchronized void writePeriodicOutputs() {
        if (state_ == DriveState.OpenLoop) {
            // TODO Set output of Spark Maxes in open loop and an 
            // feedforward of 0.0
        } else {
            // TODO Set velocity output of Spark Maxes and make sure
            // to pass it the feedforward
        }
    }

    public static class PeriodicIO {
        // INPUTS
        double left_distance;
        double right_distance;
        double left_velocity_rpm;
        double right_velocity_rpm;
        double left_velocity_mps;
        double right_velocity_mps;
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
        SmartDashboard.putNumber("Right Drive Distance", io_.right_distance);;
        SmartDashboard.putNumber("Left Drive Distance", io_.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        Pose2d odometry_pose = getOdometryPose();
        SmartDashboard.putNumber("Odometery X", odometry_pose.getTranslation().getX());
        SmartDashboard.putNumber("Odometery Y", odometry_pose.getTranslation().getY());
        SmartDashboard.putNumber("Odometery Rotation", odometry_pose.getRotation().getDegrees());


        if (CSVWriter_ != null) {
            CSVWriter_.write();
        }
    }

    // TODO: Implement system check
    @Override
    public boolean checkSystem(){
        return true;
    }
}
