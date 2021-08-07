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
import frc2020.subsystems.Shooter.ShooterState;

/**
 * Starting from center point, shoots three balls, goes straight
 * to trench, picks up three balls, drives out of the trench, on
 * center with the power port and shoots.
 */
public class NewTrenchAuto extends AutoMode {
  private static DifferentialDriveKinematics DRIVE_KINEMATICS =
      Drive.getInstance().getKinematics();
  private static SimpleMotorFeedforward FEEDFORWARD =
      Drive.getInstance().getFeedforward();

  public static Trajectory centerToTrench = null;
  public static Trajectory trenchToEnd = null;

  public static void generateTrajectories() {
    var maxVoltage = 10.0; //Volts

    var trenchPickup = new Pose2d(FieldConstants.THIRD_TRENCH_BALL_X - Constants.ROBOT_LENGTH / 2 + Constants.INTAKE_LENGTH + 0.5,
        FieldConstants.THIRD_TRENCH_BALL_Y - 0.1, new Rotation2d());
    {
      var maxAccel = 2.3; // meters/sec^2
      var maxVelocity = 2.3; // meters/sec

      var startPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + Constants.ROBOT_LENGTH,
          FieldConstants.CENTER_POWER_PORT_Y, new Rotation2d());

      Translation2d midPoint1 = startPose.transformBy(new Transform2d(
          new Translation2d(1.5875, 1.0652 + 0.06), Rotation2d.fromDegrees(0))).getTranslation();

      List<Translation2d> midPoints = List.of(
          midPoint1
      );

      var endPose = trenchPickup;

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
      var maxAccel = 3.0; // meters/sec^2
      var maxVelocity = 3.0; // meters/sec

      var startPose = trenchPickup;

      List<Translation2d> midPoints = List.of(
          // TODO: Maybe put this point back in
          //startPose.transformBy(new Transform2d(new Translation2d(-2.0, -0.5), new Rotation2d())).getTranslation()
      );

      var endPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + Constants.ROBOT_LENGTH,
          FieldConstants.CENTER_POWER_PORT_Y, new Rotation2d());

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

    runCommand(new Shoot(3.5)); // TODO: Optimize
    runCommand(new IntakeCommand(true));
    runCommand(new DriveTrajectory(centerToTrench, true));
    runCommand(new WaitCommand(0.5)); //TODO: Try removing
    runCommand(new DriveTrajectory(trenchToEnd));
    runCommand(new Shoot(3.5)); // TODO: Optimize
  }

}