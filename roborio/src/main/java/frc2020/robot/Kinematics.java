package frc2020.robot;

import frc2020.util.geometry.Pose2d;
import frc2020.util.geometry.Rotation2d;
import frc2020.util.geometry.Twist2d;

/**
 * Provides forward kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 *
 * @author Team 254
 * @author JavaDoced and modified by Team 4910
 */
public class Kinematics {
    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit. This is less accurate than using an actual gryo mesurment
     * but is useful for modeling motion.
     *
     * @param left_wheel_delta  The change in left side encoder ticks
     * @param right_wheel_delta The change in right side encoder ticks
     * @return A Twist2d that represents the change in differnce driven and the change in heading
     */
    public static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta) {
        double delta_rotation = (right_wheel_delta - left_wheel_delta) / (Constants.DRIVE_TRACK_WIDTH * Constants.TRACK_SCRUB_FACTOR);
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    /**
     * Forward kinematics using encoders and gryo.
     *
     * @param left_wheel_delta    The change in left side encoder ticks
     * @param right_wheel_delta   Then change in right side encoder ticks
     * @param delta_rotation_rads The change in heading in rads (usealy mesured by encoders)
     * @return A Twist2d that represents the distance driven and the change in heading
     */
    public static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist2d(dx, 0.0, delta_rotation_rads);
    }

    /**
     * Forward kinematics using encoders, previous heading, and current heading.
     *
     * @param prev_heading      The previous robot heading
     * @param left_wheel_delta  The change in left side encoder ticks
     * @param right_wheel_delta The change in right side encoder ticks
     * @param current_heading   The current robot heading
     */
    public static Twist2d forwardKinematics(Rotation2d prev_heading, double left_wheel_delta, double right_wheel_delta,
                                            Rotation2d current_heading) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = 0.0; // 0 For differential drive trains
        return new Twist2d(dx, dy, prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    /**
     * Integrate forward kinematics with a Twist2d and previous pose.
     *
     * @param current_pose       The current robot pose
     * @param forward_kinematics The change in robot distance and heading from the current pose
     * @return The robot pose that is accheved from adding the forward_kinematics to the current robot pose
     */
    public static Pose2d integrateForwardKinematics(Pose2d current_pose,
                                                    Twist2d forward_kinematics) {
        return current_pose.transformBy(Pose2d.exp(forward_kinematics));
    }

    /**
     * Calculates the scrub factor given a change in position AND heading
     *
     * @param delta_rotation_rads The change in rotation in rads
     * @param left_wheel_delta    The change in distance of the left drive
     * @param right_wheel_delta   The change in distance of the right drive
     * @return NaN if the change in heading is 0 or the left change and the right change are the same otherwise the scub factor
     */
    public static double claculateTrackScubFactor(double delta_rotation_rads, double left_wheel_delta, double right_wheel_delta) {
        if (delta_rotation_rads == 0)
            return 1.0;
        return (delta_rotation_rads * Constants.DRIVE_TRACK_WIDTH) / (left_wheel_delta - right_wheel_delta);
    }
}
