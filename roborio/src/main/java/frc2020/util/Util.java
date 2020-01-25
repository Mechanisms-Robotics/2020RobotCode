package frc2020.util;

import edu.wpi.first.wpilibj.geometry.*;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    // For whatever reason wpilib when porting over 254's
    // geometry classes split up the transforming
    // functionality of the Pose2d into a different class.
    // They did'nt provide any way by default to convert
    // between the two so here is some functions to do that
    /**
     * Converts a Pose2d to a Transform2d
     * @param pose The pose to convert.
     * @return The Transform2d that is equivalent to the pose
     * @see Pose2d
     * @see Transform2d
     */
    public static Transform2d poseToTransform(Pose2d pose) {
        return new Transform2d(pose.getTranslation(),
                pose.getRotation());
    }

    /**
     * Converts a Transform2d into a Pose2d
     * @param transform The transform to convert
     * @return The Pose2d that is equivalent to the pose
     * @see Pose2d
     * @see Transform2d
     */
    public static Pose2d transformToPose(Transform2d transform) {
        return new Pose2d(transform.getTranslation(),
                transform.getRotation());
    }

    /**
     * Interpolate between two Translation2d Some percent x
     * @param a The starting Translation2d
     * @param b The ending Translation2d
     * @param x The percent to interpolate to. 0.0 being a 1.0 being b
     * @return The interpolated Translation2d
     * @see Translation2d
     */
    public static Translation2d interpolateTranslation2d(Translation2d a, Translation2d b, double x) {
        if (x <= 0) {
            return new Translation2d(a.getX(), a.getY());
        } else if (x >= 1) {
            return new Translation2d(b.getX(), b.getY());
        }
        return new Translation2d(x * (b.getX() - a.getX()) + a.getX(),
                x * (b.getY() - a.getY()) + a.getY());
    }

    /**
     * Interpolate between two Rotation2ds by some percent x
     * @param a The starting Rotation2d
     * @param b The ending Rotation2d
     * @param x The percent to interpolate to. 0.0 bing a and 1.0 being b
     * @return The interpolated Rotation2d
     * @see Rotation2d
     */
    public static Rotation2d interpolateRotation2d(Rotation2d a, Rotation2d b, double x) {
        if (x <= 0.0) {
            return new Rotation2d(a.getRadians());
        } else if (x >= 1.0) {
            return new Rotation2d(b.getRadians());
        }
        double angle_diff = a.unaryMinus().rotateBy(b).getRadians();
        return a.rotateBy(new Rotation2d(angle_diff * x));
    }

    /**
     * Interpolate between two Pose2d by some x assuming constant curvature
     * @param a The starting Pose2d
     * @param b the ending Pose2d
     * @param x
     * @return The interpolated Pose2d
     * @see Pose2d
     */
    public static Pose2d interpolatePose2d(Pose2d a, Pose2d b, double x) {
        if (x <= 0.0) {
            return new Pose2d(a.getTranslation(), a.getRotation());
        } else if (x >= 1.0) {
            return new Pose2d(b.getTranslation(), b.getRotation());
        }
        Twist2d twist = a.log(b);
        twist = new Twist2d(twist.dx * x, twist.dy * x, twist.dtheta * x);
        return a.exp(twist);
    }

    public static Pose2d invertPose2d(Pose2d pose) {
        Rotation2d rotation_inverted = pose.getRotation().unaryMinus();
        return new Pose2d(pose.getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }
}