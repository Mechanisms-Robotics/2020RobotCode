package frc2020.robot;

import frc2020.util.geometry.Pose2d;
import frc2020.util.geometry.Rotation2d;
import frc2020.util.geometry.Twist2d;
import frc2020.util.InterpolatingDouble;
import frc2020.util.InterpolatingTreeMap;
import frc2020.util.Units;
import frc2020.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

/**
 * Robot State keeps track of the robot in relationship to various frames. A frame is just a
 * point and direction that defines an (x, y) cordnet system. Poses keep track of the spatial relationship
 * between different frames.
 * In the 2018 game 4910 only has robot-to-feild frame. The origen of which is the center of our alance station wall.
 *
 * @author Team 254
 * @author Modified and JavaDoced by Team 4910
 */
public class RobotState {

    // The robot-wide instance_ of Robot State
    private static RobotState instance_ = new RobotState();

    /**
     * Gets the current robot wide instance_ of robot state
     */
    public static RobotState getInstance() {
        return instance_;
    }

    // How many observations to store 
    private static final int kObservationBufferSize = 100;

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private double distance_driven_;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> range_to_target_;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> bearing_to_target_;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> skew_to_target_;

    /**
     * Private contuctor for RobotState
     */
    private RobotState() {
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     *
     * @param start_time In most casses this is just Timer.getFTPATimestamp() when you call the fuction
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        Drive.getInstance().setHeading(initial_field_to_vehicle.getRotation());
        range_to_target_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        bearing_to_target_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        skew_to_target_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        distance_driven_ = 0.0;
    }

    /**
     * Resets the field to robot's X and Y cornet on the field but not the heading
     *
     * @param start_time In most cases this is just Timer.getFTPATimestamp() when you call the function
     */
    public synchronized void resetXY(double start_time, Pose2d initial_field_to_vehicle) {
        Rotation2d current_heading = field_to_vehicle_.lastEntry().getValue().getRotation();
        initial_field_to_vehicle = new Pose2d(initial_field_to_vehicle.getTranslation(), current_heading);
      //  RobotLogger.getInstance().logInfo("Resetting position to: " + initial_field_to_vehicle);
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        Drive.getInstance().setHeading(initial_field_to_vehicle.getRotation());
        range_to_target_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        bearing_to_target_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        distance_driven_ = 0.0;
    }

    /**
     * Reset the storted distance driven
     */
    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     *
     * @param timestamp Time in secounds to lookup at. General dervied from Timer.getFGPATimestamp. (ie. to get 5 seconds earlyer Timer.getFTPATimestamp() - 5)
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    /**
     * Returns a predicted pose given that the robot remains at the current velocity.
     */
    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    /**
     * Add an observation to the robot pose in relation to the feild.
     */
    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    /**
     * Add an observation from a current volocity and a predited volocity.
     */
    public synchronized void addObservations(double timestamp, Twist2d measured_velocity,
                                             Twist2d predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    /**
     * Add a new observation from typical drive train sesors. (eg. Encoders and gyro)
     */
    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance,
                current_gyro_angle);
        distance_driven_ += delta.dx;
        return delta;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public void outputToSmartDashboard() {
        Pose2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
        SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
        SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Linear Velocity", vehicle_velocity_measured_.dx);
    }

}