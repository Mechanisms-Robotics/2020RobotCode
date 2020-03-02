package frc2020.auto.modes;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.FieldConstants;
import frc2020.auto.commands.*;
import frc2020.robot.Constants;
import frc2020.subsystems.Drive;

/**
 * Starts in front of power port, shoots 3 balls, goes to trench and takes two balls,
 * turn 90 degrees to the right, and shoots those balls.
 */
public class RightToTrench8 extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = Drive.getInstance().getFeedforward();

    public static Trajectory firstFive = null;
    public static Trajectory nextThree = null;
    public static Trajectory moveToShoot = null;

    public static void generateTrajectories() {
        var maxVoltage = 10.0; // volts

        Pose2d firstPickup = new Pose2d(FieldConstants.FIRST_TRENCH_BALL_X,
                                        FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());
        Pose2d secondPickup = new Pose2d(FieldConstants.THIRD_TRENCH_BALL_X + 0.1,
                                            FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());
        {
            var maxAccel = 2.0; // meters / seconds*2
            var maxVelocity = 2.0; // meters / seconds

            var startPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + Constants.ROBOT_LENGTH,
                                        FieldConstants.RIGHT_POSITION_Y, new Rotation2d());

            List<Translation2d> midPoints = List.of(
            );

            var endPose = firstPickup;

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            firstFive = TrajectoryGenerator.generateTrajectory(
                startPose, midPoints, endPose, config);

        }

        {
            var maxAccel = 1.5; // meters / seconds*2
            var maxVelocity = 1.5; // meters / seconds

            var startPose = firstPickup;

            List<Translation2d> midPoints = List.of(
            );

            var endPose = secondPickup;

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            nextThree = TrajectoryGenerator.generateTrajectory(
                    startPose, midPoints, endPose, config);

        }

        {
            var maxAccel = 2.5; // meters / seconds*2
            var maxVelocity = 2.5; // meters / seconds

            var startPose = secondPickup;

            List<Translation2d> midPoints = List.of(
            );

            var endPose = new Pose2d(FieldConstants.SECOND_TRENCH_BALL_X + 0.5,
                    FieldConstants.THIRD_TRENCH_BALL_Y - 0.2, Rotation2d.fromDegrees(15));

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);
            config.setReversed(true);

            moveToShoot = TrajectoryGenerator.generateTrajectory(
                    startPose, midPoints, endPose, config);

        }
            
        
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        if (firstFive == null || nextThree == null || moveToShoot == null) {
            generateTrajectories();
        }
        runCommand(new IntakeCommand(true));
        runCommand(new DriveTrajectory(firstFive, true));
        runCommand(new Shoot(5));
        runCommand(new DriveTrajectory(nextThree));
        runCommand(new WaitCommand(0.5));
        runCommand(new DriveTrajectory(moveToShoot));
        runCommand(new Shoot(5));

    }
}