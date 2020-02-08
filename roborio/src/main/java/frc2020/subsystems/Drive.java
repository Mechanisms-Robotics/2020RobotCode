package frc2020.subsystems;

import frc2020.robot.Robot;
import frc2020.util.*;
import frc2020.util.drivers.NavX;
import frc2020.loops.Loop;
import frc2020.loops.ILooper;
import frc2020.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.*;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * The base drive train class for the 2020 robot. It contains all functions
 * required to move for both autonomous and teleop periods of the game.
 * Path following logic inspired by 254
 *
 * @author Team 4910
 */
public class Drive implements Subsystem {

    private static final double DRIVE_ENCODER_PPR = 4096.0;
    private static final double NEO_ENCODER_PPR = 46.0;
    private static final double HIGH_GEAR_RATIO = 8.63;

    private static final int VELOCITY_PID = 0;
    private static final int EMPTY_PID = 1;

    private static Logger logger_ = Logger.getInstance();

    private static final String logName = "Drive";
    
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
    private CANSparkMax leftMaster_;
    private CANSparkMax leftSlave_;
    private CANSparkMax rightMaster_;
    private CANSparkMax rightSlave_;

    // Moter PIDs
    private CANPIDController leftVelocityPID_;
    private CANPIDController rightVelocityPID_;

    // Encoders and Odometry
    private CANCoder leftCanCoder;
    private CANCoder rightCanCoder;

    private CANCoderConfiguration leftCoderConfig;
    private CANCoderConfiguration rightCoderConfig;

    private DifferentialDriveOdometry odometry_;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingPose2d> odometryHistory;

    // Shifter
    private DoubleSolenoid shifter_;
    private DoubleSolenoid.Value lowGear_ = DoubleSolenoid.Value.kReverse;
    private DoubleSolenoid.Value highGear_ = DoubleSolenoid.Value.kForward;

    // IO
    // This is to prevent overwhelming the CAN Bus / HAL
    private PeriodicIO io_;

    private ReflectingCSVWriter<PeriodicIO> CSVWriter_ = null;

    // Path Following
    private DifferentialDriveKinematics kinematics_;
    private Trajectory currentTrajectory_;
    private double trajectoryStartTime_;
    private RamseteController controller_;
    private SimpleMotorFeedforward feedforward_;
    private boolean doneWithTrajectory_;

    private boolean IS_ROBOT = false;

    /**
     * The default constructor starts the drive train and sets it up to be in
     * OpenLoop mode
     */
    private Drive() {
        if (Robot.isReal() && IS_ROBOT) {
            configSparkMaxs();
            configCanCoders();

            // Configure NavX
            gyro_ = new NavX(SerialPort.Port.kUSB);
            setBrakeMode(true); //beginning brake mode (can be changed by CSGenerator)

            shifter_ = new DoubleSolenoid(Constants.SHIFT_FORWARD, Constants.SHIFT_REVERSE);
        }
        odometry_ = new DifferentialDriveOdometry(new Rotation2d(0.0));
        odometryHistory = new InterpolatingTreeMap<>(100);

        kinematics_ = new DifferentialDriveKinematics(
                Constants.TRACK_SCRUB_FACTOR * Constants.DRIVE_TRACK_WIDTH
        );
        currentTrajectory_ = null;
        trajectoryStartTime_ = -1.0;
        doneWithTrajectory_ = true;
        feedforward_ = new SimpleMotorFeedforward(
                Constants.DRIVE_V_INTERCEPT,
                Constants.DRIVE_KV,
                Constants.DRIVE_KA
        );
        controller_ = new RamseteController();
        io_ = new PeriodicIO();
    }

    /**
     * Loads the CAN Coder Configuration files to the CAN Coders. Also reports magnet alignment and config errors
     * Encoder polarity, unit coefficients also managed here
     */
    private void configCanCoders() {
        leftCanCoder = new CANCoder(Constants.LEFT_CAN_CODER_ID);
        rightCanCoder = new CANCoder(Constants.RIGHT_CAN_CODER_ID);

        // Reports firmware version for logging purposes
        logger_.logInfo("Left CAN Coder Firmware: " + leftCanCoder.getFirmwareVersion(), logName);
        logger_.logInfo("Right CAN Coder Firmware: " + rightCanCoder.getFirmwareVersion(), logName);
        
        // Checks for alignment of magnet with encoder (the encoder light color)
        MagnetFieldStrength leftMagStrength = leftCanCoder.getMagnetFieldStrength();
        if (leftMagStrength == MagnetFieldStrength.BadRange_RedLED) {
            logger_.logError("Left CAN Coder magnet in the red (out of range)", logName);
        } else if (leftMagStrength == MagnetFieldStrength.Adequate_OrangeLED) {
            logger_.logWarning("Left CAN Coder magnet in the orange (slightly out of alignment)", logName);
        } else if (leftMagStrength == MagnetFieldStrength.Good_GreenLED) {
            logger_.logDebug("Left CAN Coder magnet is green (healthy)", logName);
        } else {
            logger_.logError("Left CAN Coder magnet not detected", logName);
        }

        MagnetFieldStrength rightMagStrength = rightCanCoder.getMagnetFieldStrength();
        if (rightMagStrength == MagnetFieldStrength.BadRange_RedLED) {
            logger_.logError("Right CAN Coder magnet in the red (out of range)", logName);
        } else if (rightMagStrength == MagnetFieldStrength.Adequate_OrangeLED) {
            logger_.logWarning("Right CAN Coder magnet in the orange (slightly out of alignment)", logName);
        } else if (rightMagStrength == MagnetFieldStrength.Good_GreenLED) {
            logger_.logDebug("Right CAN Coder magnet is green (healthy)", logName);
        } else {
            logger_.logError("Right CAN Coder magnet not detected", logName);
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
            logger_.logError("Left CAN coder config failed with error: " + rv.toString(), logName);
        }

        rv = rightCanCoder.configAllSettings(rightCoderConfig, Constants.CAN_TIMEOUT);
        if (rv != ErrorCode.OK) {
            logger_.logError("Right CAN coder config failed with error: " + rv.toString(), logName);
        }
    }

    /**
     * Configures spark maxes by assigning port number and motor type, restores the factory
     * default settings, sets inversions because one motor is usually flipped, sets ramp
     * rates for ramping up NEOs, sets the frame period and frame rate sent, and velocity
     * PIDs
     */
    private void configSparkMaxs() {
        leftMaster_ = new CANSparkMax(Constants.LEFT_MASTER_PORT, MotorType.kBrushless);
        leftSlave_ = new CANSparkMax(Constants.LEFT_SLAVE_PORT, MotorType.kBrushless);
        rightMaster_ = new CANSparkMax(Constants.RIGHT_MASTER_PORT, MotorType.kBrushless);
        rightSlave_ = new CANSparkMax(Constants.RIGHT_SLAVE_PORT, MotorType.kBrushless);
        
        leftMaster_.restoreFactoryDefaults();
        leftSlave_.restoreFactoryDefaults();
        rightMaster_.restoreFactoryDefaults();
        rightSlave_.restoreFactoryDefaults();
        
        leftMaster_.setInverted(true);
        rightMaster_.setInverted(false);

        leftSlave_.follow(leftMaster_);
        rightSlave_.follow(rightMaster_);

        leftMaster_.setOpenLoopRampRate(Constants.OPEN_LOOP_RAMP);
        leftMaster_.setClosedLoopRampRate(Constants.CLOSED_LOOP_RAMP);
        leftMaster_.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        leftMaster_.setSmartCurrentLimit(Constants.STALL_LIMIT, Constants.FREE_LIMIT);
        
        rightMaster_.setOpenLoopRampRate(Constants.OPEN_LOOP_RAMP);
        rightMaster_.setClosedLoopRampRate(Constants.CLOSED_LOOP_RAMP);
        rightMaster_.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        rightMaster_.setSmartCurrentLimit(Constants.STALL_LIMIT, Constants.FREE_LIMIT);
        
        leftVelocityPID_ = leftMaster_.getPIDController();
        rightVelocityPID_ = rightMaster_.getPIDController();
        
        leftVelocityPID_.setP(Constants.VELOCITY_HIGH_GEAR_KP, VELOCITY_PID);
        leftVelocityPID_.setI(Constants.VELOCITY_HIGH_GEAR_KI, VELOCITY_PID);
        leftVelocityPID_.setD(Constants.VELOCITY_HIGH_GEAR_KD, VELOCITY_PID);
        leftVelocityPID_.setIZone(Constants.VELOCITY_HIGH_GEAR_I_ZONE, VELOCITY_PID);
        leftVelocityPID_.setFF(Constants.VELOCITY_HIGH_GEAR_KF, VELOCITY_PID);
        rightVelocityPID_.setP(Constants.VELOCITY_HIGH_GEAR_KP, VELOCITY_PID);
        rightVelocityPID_.setI(Constants.VELOCITY_HIGH_GEAR_KI, VELOCITY_PID);
        rightVelocityPID_.setD(Constants.VELOCITY_HIGH_GEAR_KD, VELOCITY_PID);
        rightVelocityPID_.setIZone(Constants.VELOCITY_HIGH_GEAR_I_ZONE, VELOCITY_PID);
        rightVelocityPID_.setFF(Constants.VELOCITY_HIGH_GEAR_KF, VELOCITY_PID);
    }

    /**
     * getter
     */
    public DifferentialDriveKinematics getKinematics() {
        return kinematics_;
    }

    /**
     * getter
     */
    public SimpleMotorFeedforward getFeedforward() {
        return feedforward_;
    }

    /**
     * Handles the periodic function for the drive train
     */
    private final Loop DriveLoop = new Loop() {

        DriveState last_state = null;

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
                if (last_state != state_) {
                    logger_.logInfo("Drive State Changed to " + state_.toString(), logName);
                    last_state = state_;
                }
                switch (state_) {
                    case OpenLoop:
                        break;
                    case Trajectory_Following:
                        updateTrajectoryFollower();
                        break;
                    case Velocity:
                        break;
                    default:
                        logger_.logWarning("Invalid drive state_: " + state_, logName);
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
     * In order to calculate how to run our trajectory, we constantly use odometry values
     * to update how we should proceed to our trajectory endpoint
     */
    private void updateTrajectoryFollower() {
        if (!doneWithTrajectory_) {

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

                    // Calculate feedforward
                    double feedforward = feedforward_.calculate(end_speed);
                    driveVelocity(new DriveSignal(end_speed, end_speed),
                                    new DriveSignal(feedforward, feedforward));
                    state_ = DriveState.Velocity;
                    doneWithTrajectory_ = true;
                }
            } else {
                logger_.logError(
                    "Unable to follow trajectory due to invaded start time or trajectory", logName);
                openLoop(new DriveSignal(0, 0, true));
            }
        } else {
            logger_.logWarning(
                "Update path follower called when tarjectory has already been completed", logName);
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
        if (Robot.isReal() && IS_ROBOT) {
            // ADD Brake mode cofiction for SPARK MAX
            if (mode) {
                leftMaster_.setIdleMode(CANSparkMax.IdleMode.kBrake);
                rightMaster_.setIdleMode(CANSparkMax.IdleMode.kBrake);
            } else {
                leftMaster_.setIdleMode(CANSparkMax.IdleMode.kCoast);
                rightMaster_.setIdleMode(CANSparkMax.IdleMode.kCoast);
            }
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
        if (state_ != DriveState.Velocity && state_ != DriveState.Trajectory_Following) {
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
        if (state_ != DriveState.Velocity && state_ != DriveState.Trajectory_Following) {
            state_ = DriveState.Velocity;
        }
        setBrakeMode(signal.getBrakeMode());
        io_.left_demand = metersPerSecondToRpm(signal.getLeft()) * HIGH_GEAR_RATIO;
        io_.right_demand = metersPerSecondToRpm(signal.getRight()) * HIGH_GEAR_RATIO;
        io_.left_feedforward = feedforward_.calculate(signal.getLeft());
        io_.right_feedforward = feedforward_.calculate(signal.getRight());
    }
    /**
     * Drive the robot autonomously on a pre-defined trajectory
     * @param trajectory The trajectory to follow
     */
    public synchronized void driveTrajectory(Trajectory trajectory) {
        state_ = DriveState.Trajectory_Following;
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

    public synchronized void autoSteer(Rotation2d targetBearing, double baseDutyCycle) {
        double error = targetBearing.getDegrees();
        double kP = 0.02;

        double adjustedDutyCycle = kP * error;
        baseDutyCycle = Util.limit(baseDutyCycle, -0.75, 0.75);

        DriveSignal autoSteerSignal = new DriveSignal(baseDutyCycle + adjustedDutyCycle,
                                                      baseDutyCycle - adjustedDutyCycle, true);
        openLoop(autoSteerSignal);
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

        if (Robot.isReal() && IS_ROBOT) {
            // Prints the stack if there are any errors in setting the position of the CAN Coders
            ErrorCode rv = leftCanCoder.setPosition(0, Constants.CAN_TIMEOUT);
            if (rv != ErrorCode.OK) {
                logger_.logError("Left CAN coder reset failed with error: " + rv.toString(), logName);
            }

            rv = rightCanCoder.setPosition(0, Constants.CAN_TIMEOUT);
            if (rv != ErrorCode.OK) {
                logger_.logError("Right CAN coder reset failed with error: " + rv.toString(), logName);
            }

            // Sets the gyroscope to the desired rotation
            setHeading(rotation);
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
        if (Robot.isReal()) {
            if (shifter_.get() == highGear_) {
                shifter_.set(lowGear_);
            } else {
                shifter_.set(highGear_);
            }
        }
    }

    /**
     * Sets drive to high gear
     */
    public synchronized void setHighGear() {
        if (Robot.isReal() && IS_ROBOT) {
            shifter_.set(highGear_);
        }
    }

    /**
     * Sets drive to low gear
     */
    public synchronized void setLowGear() {
        if (Robot.isReal() && IS_ROBOT) {
            shifter_.set(lowGear_);
        }
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
        if (Robot.isReal()) {
            gyro_.setAngleAdjustment(adjustment);
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

    public Pose2d getOdometryPose(double timestamp) {
        return odometryHistory.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public Pose2d getOdometryPose() {
        return odometry_.getPoseMeters();
    }

    /**
    * Handles reading all of the data from encoders/CANTalons periodically. Also
    * adds these readings to the CSVWriter
    */
    public synchronized void readPeriodicInputs() {
        if (Robot.isReal() && IS_ROBOT) {
            // Get this from the CAN coders
            io_.left_distance = leftCanCoder.getPosition();
            io_.right_distance = rightCanCoder.getPosition();
            io_.left_neo_distance = rotationsToMeters(leftMaster_.getEncoder().getPosition());
            io_.right_neo_distance = rotationsToMeters(rightMaster_.getEncoder().getPosition());

            // Get this from the Spark Maxes
            io_.left_velocity_rpm = leftMaster_.getEncoder().getVelocity() / HIGH_GEAR_RATIO;
            io_.right_velocity_rpm = rightMaster_.getEncoder().getVelocity() / HIGH_GEAR_RATIO;
            io_.left_velocity_mps = leftCanCoder.getVelocity();
            io_.right_velocity_mps = rightCanCoder.getVelocity();
            io_.gyro_heading = gyro_.getYaw();

            io_.left_temperature = leftMaster_.getMotorTemperature();
            io_.right_temperature = rightMaster_.getMotorTemperature();

            odometry_.update(io_.gyro_heading, getLeftEncoderDistance(), getRightEncoderDistance());
            double timestamp = Timer.getFPGATimestamp();
            odometryHistory.put(new InterpolatingDouble(timestamp),
                    new InterpolatingPose2d(odometry_.getPoseMeters()));
        }

        if (CSVWriter_ != null) {
            CSVWriter_.add(io_);
        }
    }

    /**
    * Handles writing outputs to Spark Maxes periodically
    */
    public synchronized void writePeriodicOutputs() {
        if (Robot.isReal() && IS_ROBOT) {
            if (state_ == DriveState.OpenLoop) {
                leftVelocityPID_.setReference(io_.left_demand, ControlType.kDutyCycle, EMPTY_PID, io_.left_feedforward);
                rightVelocityPID_.setReference(io_.right_demand, ControlType.kDutyCycle, EMPTY_PID, io_.right_feedforward);
            } else {
                leftVelocityPID_.setReference(io_.left_demand, ControlType.kVelocity, VELOCITY_PID, io_.left_feedforward);
                rightVelocityPID_.setReference(io_.right_demand, ControlType.kVelocity, VELOCITY_PID, io_.right_feedforward);
            }
        }
    }

    /**
     * Important variables that are constantly updated, retrieved, set
     * displayed, and more. These variables all are tracked in one way
     * or another and we use these to adjust our speeds, trajectories,
     * loads, etc.
     */
    public static class PeriodicIO {
        // INPUTS
        double left_distance;
        double left_neo_distance;
        double right_distance;
        double right_neo_distance;
        double left_velocity_rpm;
        double right_velocity_rpm;
        double left_velocity_mps;
        double right_velocity_mps;
        double left_temperature;
        double right_temperature;
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
        SmartDashboard.putNumber("Left Drive Distance", io_.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());
        SmartDashboard.putNumber("Left CANCoder MPS", io_.left_velocity_mps);
        SmartDashboard.putNumber("Right CANCoder MPS", io_.right_velocity_mps);
        SmartDashboard.putNumber("Left NEO Distance", io_.left_neo_distance);
        SmartDashboard.putNumber("Right NEO Distance", io_.right_neo_distance);

        SmartDashboard.putNumber("Left Master Temperature: ", io_.left_temperature);
        SmartDashboard.putNumber("Right Master Temperature: ", io_.right_temperature);

        Pose2d odometry_pose = getOdometryPose();
        SmartDashboard.putNumber("Odometery X", odometry_pose.getTranslation().getX());
        SmartDashboard.putNumber("Odometery Y", odometry_pose.getTranslation().getY());
        SmartDashboard.putNumber("Odometery Rotation", odometry_pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Left FeedForward", io_.left_feedforward);
        SmartDashboard.putNumber("Right FeedForward", io_.right_feedforward);


        if (CSVWriter_ != null) {
            CSVWriter_.write();
        }
    }

    @Override
    public boolean runPassiveTests() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean runActiveTests() {
        // TODO Auto-generated method stub
        return false;
    }
}
