package frc2020.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * All of the constants of the field such as field element locations, dimensions, heights, etc.
 * Often used for creating auto paths
 */
public class FieldConstants {
    // All units in SI unless specified otherwise
    // 0,0 is defined as the center of the opposing Alliance Wall, where our Power Port is
    // 0 radians is defined as facing our Power Port, standard unit circle definitions of rotational polarity

    public final static double HALF_FIELD_WIDTH = 4.105275;
    public final static double FIELD_LENGTH = 15.98295;

    public final static double CENTER_POWER_PORT_Y = 1.692275;
    public final static double ALLIANCE_WALL_TO_INITIATION_X = 3.048; //nearest edge
    public final static double SHIELD_GENERATOR_ANGLE = 22.5; //degrees

    public final static double DISTANCE_BETWEEN_BALLS = 0.4191;
    public final static double BALL_ONE_X = ALLIANCE_WALL_TO_INITIATION_X + 3.0861;
    public final static double BALL_ONE_Y = -0.3302;
    public final static Pose2d BALL_ONE_POSE = new Pose2d(BALL_ONE_X, BALL_ONE_Y, Rotation2d.fromDegrees(SHIELD_GENERATOR_ANGLE + 90));
    
    public final static double BALL_TWO_X = BALL_ONE_POSE.transformBy(new Transform2d(
        new Translation2d(DISTANCE_BETWEEN_BALLS, 0), new Rotation2d())).getTranslation().getX();
    public final static double BALL_TWO_Y = BALL_ONE_POSE.transformBy(new Transform2d(
        new Translation2d(DISTANCE_BETWEEN_BALLS, 0), new Rotation2d())).getTranslation().getY();
    public final static Pose2d BALL_TWO_POSE = new Pose2d(BALL_TWO_X, BALL_TWO_Y, Rotation2d.fromDegrees(SHIELD_GENERATOR_ANGLE + 90));
    
    public final static double BALL_THREE_X = BALL_ONE_POSE.transformBy(new Transform2d(
        new Translation2d(DISTANCE_BETWEEN_BALLS, 0), new Rotation2d())).getTranslation().getX();
    public final static double BALL_THREE_Y = BALL_ONE_POSE.transformBy(new Transform2d(
        new Translation2d(2*DISTANCE_BETWEEN_BALLS, 0), new Rotation2d())).getTranslation().getY();
    public final static Pose2d BALL_THREE_POSE = new Pose2d(BALL_THREE_X, BALL_THREE_Y, Rotation2d.fromDegrees(SHIELD_GENERATOR_ANGLE + 90));
    
    public final static double BALL_FOUR_X = 5.9944;
    public final static double BALL_FOUR_Y = HALF_FIELD_WIDTH - 3.0607;
    public final static Pose2d BALL_FOUR_POSE = new Pose2d(BALL_FOUR_X, BALL_FOUR_Y, Rotation2d.fromDegrees(SHIELD_GENERATOR_ANGLE));
    
    public final static double BALL_FIVE_X = BALL_FOUR_POSE.transformBy(new Transform2d(
        new Translation2d(DISTANCE_BETWEEN_BALLS, 0), new Rotation2d())).getTranslation().getX();
    public final static double BALL_FIVE_Y = BALL_FOUR_POSE.transformBy(new Transform2d(
        new Translation2d(DISTANCE_BETWEEN_BALLS, 0), new Rotation2d())).getTranslation().getY();
    public final static Pose2d BALL_FIVE_POSE = new Pose2d(BALL_FIVE_X, BALL_FIVE_Y, Rotation2d.fromDegrees(SHIELD_GENERATOR_ANGLE));


    public final static double THIRD_TRENCH_BALL_X = ALLIANCE_WALL_TO_INITIATION_X + 4.943602;
    public final static double THIRD_TRENCH_BALL_Y = 3.400425;
    public final static double FOURTH_TRENCH_BALL_X = ALLIANCE_WALL_TO_INITIATION_X + 6.57606;
    public final static double RIGHT_POSITION_Y = THIRD_TRENCH_BALL_Y;

    public final static double FIRST_TRENCH_BALL_X = THIRD_TRENCH_BALL_X - 1.8288;
    public final static double SECOND_TRENCH_BALL_X = THIRD_TRENCH_BALL_X - 0.9144;
}