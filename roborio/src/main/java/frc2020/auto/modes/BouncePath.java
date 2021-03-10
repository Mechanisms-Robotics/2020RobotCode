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

public class BouncePath extends AutoMode {
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
    var maxAccel = 1.9; // m/s
    var maxVelocity = 1.9; // m/s

    // Define other constraints
    var voltageConstraint =
        new DifferentialDriveVoltageConstraint(
            FEEDFORWARD,
            DRIVE_KINEMATICS,
            maxVoltage);

    var config1 =
        new TrajectoryConfig(maxVelocity,
            maxAccel)
            .setKinematics(DRIVE_KINEMATICS)
            .addConstraint(voltageConstraint);

    var config2 =
        new TrajectoryConfig(maxVelocity,
            maxAccel)
            .setKinematics(DRIVE_KINEMATICS)
            .addConstraint(voltageConstraint)
            .setReversed(true);

    {
      var waypoints = new ArrayList<Pose2d>();

      var waypoint1 = new Pose2d(0.0, 0.0, new Rotation2d());
      var waypoint2 = new Pose2d(0.762, 0.0, new Rotation2d());
      var waypoint3 = new Pose2d(1.524, 1.724, Rotation2d.fromDegrees(90.0));
      var waypoint4 = new Pose2d(1.514, 1.424, Rotation2d.fromDegrees(-90.0));
      var waypoint5 = new Pose2d(1.886, -0.181, Rotation2d.fromDegrees(-45.0));
      var waypoint6 = new Pose2d(3.048, -1.524, new Rotation2d());
      var waypoint7 = new Pose2d(4.21, -0.762, Rotation2d.fromDegrees(90.0));
      var waypoint8 = new Pose2d(4.21, 1.724, Rotation2d.fromDegrees(90.0));
      var waypoint9 = new Pose2d(4.11, 1.424, Rotation2d.fromDegrees(-90.0));
      var waypoint10 = new Pose2d(4.21, -0.762, Rotation2d.fromDegrees(-90.0));
      var waypoint11 = new Pose2d(5.334, -1.524, new Rotation2d());
      var waypoint12 = new Pose2d(6.096, -1.524, new Rotation2d());
      var waypoint13 = new Pose2d(6.658, -0.762, Rotation2d.fromDegrees(90.0));
      var waypoint14 = new Pose2d(6.758, 1.724, Rotation2d.fromDegrees(90.0));
      var waypoint15 = new Pose2d(6.758, 1.524, Rotation2d.fromDegrees(-80.0));
      var waypoint16 = new Pose2d(6.758, 0.762, Rotation2d.fromDegrees(-90.0));
      var waypoint17 = new Pose2d(7.858, 0.0, Rotation2d.fromDegrees(-20.0));

      waypoints.add(waypoint1);
      waypoints.add(waypoint2);
      waypoints.add(waypoint3);
      waypoints.add(waypoint4);
      waypoints.add(waypoint5);
      waypoints.add(waypoint6);
      waypoints.add(waypoint7);
      waypoints.add(waypoint8);
      waypoints.add(waypoint9);
      waypoints.add(waypoint10);
      waypoints.add(waypoint11);
      waypoints.add(waypoint12);
      waypoints.add(waypoint13);
      waypoints.add(waypoint14);
      waypoints.add(waypoint15);
      waypoints.add(waypoint16);
      waypoints.add(waypoint17);

      part1 = TrajectoryGenerator.generateTrajectory(waypoints, config1);
    }

    {
      var waypoints = new ArrayList<Pose2d>();

      var waypoint1 = new Pose2d(1.524, 1.424, Rotation2d.fromDegrees(-90.0));
      var waypoint2 = new Pose2d(1.886, -0.181, Rotation2d.fromDegrees(-45.0));
      var waypoint3 = new Pose2d(3.048, -1.524, new Rotation2d());
      var waypoint4 = new Pose2d(4.21, -0.762, Rotation2d.fromDegrees(90.0));
      var waypoint5 = new Pose2d(4.21, 1.724, Rotation2d.fromDegrees(90.0));

      waypoints.add(waypoint1);
      waypoints.add(waypoint2);
      waypoints.add(waypoint3);
      waypoints.add(waypoint4);
      waypoints.add(waypoint5);

      part2 = TrajectoryGenerator.generateTrajectory(waypoints, config1);
    }

    {
      var waypoints = new ArrayList<Pose2d>();

      var waypoint1 = new Pose2d(4.21, 1.424, Rotation2d.fromDegrees(-90.0));
      var waypoint2 = new Pose2d(4.21, -0.762, Rotation2d.fromDegrees(-90.0));
      var waypoint3 = new Pose2d(5.334, -1.524, new Rotation2d());
      var waypoint4 = new Pose2d(6.096, -1.524, new Rotation2d());
      var waypoint5 = new Pose2d(6.558, -0.762, Rotation2d.fromDegrees(90.0));
      var waypoint6 = new Pose2d(6.758, 1.724, Rotation2d.fromDegrees(90.0));

      waypoints.add(waypoint1);
      waypoints.add(waypoint2);
      waypoints.add(waypoint3);
      waypoints.add(waypoint4);
      waypoints.add(waypoint5);
      waypoints.add(waypoint6);

      part3 = TrajectoryGenerator.generateTrajectory(waypoints, config1);
    }

    {
      var waypoints = new ArrayList<Pose2d>();

      var waypoint1 = new Pose2d(6.758, 1.524, Rotation2d.fromDegrees(-80.0));
      var waypoint2 = new Pose2d(6.758, 0.362, Rotation2d.fromDegrees(-90.0));
      var waypoint3 = new Pose2d(8.382, 0.0, new Rotation2d());

      waypoints.add(waypoint1);
      waypoints.add(waypoint2);
      waypoints.add(waypoint3);

      part4 = TrajectoryGenerator.generateTrajectory(waypoints, config1);

    }
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (part1 == null || part2 == null || part3 == null || part4 == null) {
      generateTrajectories();
    }

    runCommand(new DriveTrajectory(part1, true));
    // runCommand(new DriveTrajectory(part2));
    // runCommand(new DriveTrajectory(part3));
    // runCommand(new DriveTrajectory(part4));
  }
}