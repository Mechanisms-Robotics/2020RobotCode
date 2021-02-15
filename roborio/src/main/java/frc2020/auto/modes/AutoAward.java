package frc2020.auto.modes;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeEndedException;
import frc2020.subsystems.Drive;
import frc2020.auto.commands.*;

import java.util.ArrayList;

public class AutoAward extends AutoMode {
  private static DifferentialDriveKinematics DRIVE_KINEMATICS =
      Drive.getInstance().getKinematics();
  private static SimpleMotorFeedforward FEEDFORWARD =
      Drive.getInstance().getFeedforward();

  private static Trajectory part1 = null;
  private static Trajectory part2 = null;
  private static Trajectory part3 = null;
  private static Trajectory part4 = null;

  public static void generateTrajectories() {
    var maxVoltage = 10.0; // Volts
    var maxAccel = 1.5; // m/s
    var maxVelocity = 1.5; // m/s

    // Define other constraints
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

    {
      var startPose = new Pose2d();

      var midPoints = new ArrayList<Translation2d>();

      var midPoint1 = new Translation2d(0.762, 0.0);

      midPoints.add(midPoint1);

      var endPose = new Pose2d(1.524, 0.762, Rotation2d.fromDegrees(45.0));

      part1 = TrajectoryGenerator.generateTrajectory(startPose, midPoints, endPose, config);
    }

    {
      var startPose = new Pose2d(1.524, 0.762, Rotation2d.fromDegrees(45.0));

      var midPoints = new ArrayList<Translation2d>();

      var midPoint1 = new Translation2d(2.286, 1.524);
      var midPoint2 = new Translation2d(3.048, 1.524);

      midPoints.add(midPoint1);
      midPoints.add(midPoint2);

      var endPose = new Pose2d(3.81, 1.524, new Rotation2d());

      part2 = TrajectoryGenerator.generateTrajectory(startPose, midPoints, endPose, config);
    }

    {
      var startPose = new Pose2d(3.81, 1.524, new Rotation2d());

      var midPoints = new ArrayList<Translation2d>();

      var midPoint1 = new Translation2d(4.572, 0.762);
      var midPoint2 = new Translation2d(5.344, 0.0);
      var midPoint3 = new Translation2d(6.096, 0.762);
      var midPoint4 = new Translation2d(6.096, 2.286);

      midPoints.add(midPoint1);
      midPoints.add(midPoint2);
      midPoints.add(midPoint3);
      midPoints.add(midPoint4);

      var endPose = new Pose2d(4.334, 2.286, Rotation2d.fromDegrees(180.0));

      part3 = TrajectoryGenerator.generateTrajectory(startPose, midPoints, endPose, config);
    }

    {
      var startPose = new Pose2d(4.334, 2.286, Rotation2d.fromDegrees(180.0));

      var midPoints = new ArrayList<Translation2d>();

      var midPoint1 = new Translation2d(2.286, 2.286);

      midPoints.add(midPoint1);

      var endPose = new Pose2d(3.048, 1.524, new Rotation2d());

      part4 = TrajectoryGenerator.generateTrajectory(startPose, midPoints, endPose, config);
    }
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (part1 == null || part2 == null || part3 == null || part4 == null) {
      generateTrajectories();
    }
    runCommand(new Aim());
    runCommand(new IntakeCommand(true));
    runCommand(new DriveTrajectory(part1));
    runCommand(new ShootWhileDriving());
    runCommand(new DriveTrajectory(part2));
    runCommand(new WaitCommand(3));
    runCommand(new StowShooter());
    runCommand(new DriveTrajectory(part3));
    runCommand(new IntakeCommand(false));
    runCommand(new DriveTrajectory(part4));
    runCommand(new Shoot(3));
  }
}