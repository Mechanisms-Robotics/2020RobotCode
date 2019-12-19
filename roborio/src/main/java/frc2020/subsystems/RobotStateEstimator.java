package frc2020.subsystems;

import frc2020.util.geometry.Rotation2d;
import frc2020.util.geometry.Pose2d;
import frc2020.loops.ILooper;
import frc2020.loops.Loop;
import frc2020.loops.Looper;
import frc2020.util.geometry.Twist2d;
import frc2020.robot.Kinematics;

import frc2020.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class RobotStateEstimator implements Subsystem {
    static RobotStateEstimator instance_ = new RobotStateEstimator();
    private RobotState robot_state_ = RobotState.getInstance();
    private Drive drive_ = Drive.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;

    private RobotStateEstimator() {
    }

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public void readPeriodicInputs() {
        // Nothing To do here
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
        // Nothing to do here
    }

    @Override
    public void stop() {
        // Nothing to do here
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        enabledLooper.register(Integrater);
    }

    @Override
    public void outputTelemetry() {
        RobotState.getInstance().outputToSmartDashboard();
    }

    /**
     * Handles the periodic function for the State Estimator
     */
    private final Loop Integrater = new Loop() {

        /**
         * Handles any enabling tasks for the drive train.
         */
        public void init() {
            synchronized (RobotStateEstimator.this) {
                left_encoder_prev_distance_ = drive_.getLeftEncoderDistance();
                right_encoder_prev_distance_ = drive_.getRightEncoderDistance();
            }
        }

        /**
         * The function that handles any periodic tasks that the State Estimator needs.
         */
        public void run() {
            synchronized (RobotStateEstimator.this) {
                double timestamp = Timer.getFPGATimestamp();

                // Update Robot Position
                final double left_distance = drive_.getLeftEncoderDistance();
                final double right_distance = drive_.getRightEncoderDistance();
                final double delta_left = left_distance - left_encoder_prev_distance_;
                final double delta_right = right_distance - right_encoder_prev_distance_;
                final Rotation2d gyro_angle = drive_.getHeading();
                final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(delta_left, delta_right,
                        gyro_angle);
                final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftLinearVelocity(),
                        drive_.getRightLinearVelocity());
                robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
                left_encoder_prev_distance_ = left_distance;
                right_encoder_prev_distance_ = right_distance;

            }
        }

        /**
         * Handles any tasks for the state estimator on disabling
         */
        public void end() {
            // Nothing to do
        }

    };

}

