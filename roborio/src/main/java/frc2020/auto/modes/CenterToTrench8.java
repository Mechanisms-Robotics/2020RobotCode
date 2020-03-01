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
import frc2020.auto.*;
import frc2020.auto.commands.*;
import frc2020.robot.Constants;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Shooter;

/**
 * Starting from same point as basic13Ball, shoots three balls, goes straight
 * to trench, takes up three balls, turns around and goes next to shield
 * generator column closest to power port, and shoots.
 */
public class CenterToTrench8 extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS =
            Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD =
            Drive.getInstance().getFeedforward();

    public static Trajectory centerToTrench = null;
    public static Trajectory trenchToEnd = null;

    public static void generateTrajectories() {
        var maxVoltage = 10.0; //Volts
        var maxAccel = 1.0; // meters/sec^2
        var maxVelocity = 1.0; // meters/sec
        {
            var startPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + Constants.ROBOT_LENGTH,
                    FieldConstants.CENTER_POWER_PORT_Y, new Rotation2d());

            Translation2d midPoint1 = startPose.transformBy(new Transform2d(
                    new Translation2d(1.5875, 1.0652), Rotation2d.fromDegrees(0))).getTranslation();

            List<Translation2d> midPoints = List.of(
                    midPoint1
            );

            var endPose = new Pose2d(FieldConstants.THIRD_TRENCH_BALL_X - Constants.ROBOT_LENGTH / 2 + Constants.INTAKE_LENGTH + .1, FieldConstants.THIRD_TRENCH_BALL_Y - 0.1, new Rotation2d());

            var voltageConstraint =
                    new DifferentialDriveVoltageConstraint(
                            FEEDFORWARD,
                            DRIVE_KINEMATICS,
                            maxVoltage);

            var config =
                    new TrajectoryConfig(maxVelocity,
                            maxAccel)
                            .setKinematics(DRIVE_KINEMATICS)
                            .addConstraint(voltageConstraint);

            centerToTrench = TrajectoryGenerator.generateTrajectory(
                    startPose, midPoints, endPose, config);
        }

        {

            var startPose = new Pose2d(FieldConstants.THIRD_TRENCH_BALL_X, FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());

            List<Translation2d> midPoints = List.of(
                //new Translation2d(2.471801‬, 0.854075‬)
            );

            var endPose = startPose.transformBy(new Transform2d(new Translation2d(-2.54, -1.6256), Rotation2d.fromDegrees(45)));

            var voltageConstraint =
                new DifferentialDriveVoltageConstraint(
                    FEEDFORWARD,
                    DRIVE_KINEMATICS,
                    maxVoltage);

            var config =
                new TrajectoryConfig(maxVelocity,
                                    maxAccel)
                    .setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);
            config.setReversed(true);

            trenchToEnd = TrajectoryGenerator.generateTrajectory(
                startPose, midPoints, endPose, config);
        }
    }

        @Override
        protected void routine() throws AutoModeEndedException {
            if (centerToTrench == null   || trenchToEnd == null) {
                generateTrajectories();
            }

            runCommand(new Shoot(5.0));
            runCommand(new IntakeCommand(true));
            runCommand(new DriveTrajectory(centerToTrench, true));
            runCommand(new IntakeCommand(false));
            runCommand(new Shoot(5.0));

            //runCommand(new WaitCommand(0.5));
            //runCommand(new DriveTrajectory(trenchToEnd));
        }

}