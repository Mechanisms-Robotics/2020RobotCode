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

  public static void generateTrajectories() {
    var maxVoltage = 10.0; // Volts
    var maxAccel = 1.0; // m/s
    var maxVelocity = 1.0; // m/s

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
      var waypoints = new ArrayList<Pose2d>();

      var waypoint1 = new Pose2d(0.0, 0.0, new Rotation2d());
      var waypoint2 = new Pose2d(2.286, 1.524, new Rotation2d());
      var waypoint3 = new Pose2d(3.81, 1.524, new Rotation2d());

      waypoints.add(waypoint1);
      waypoints.add(waypoint2);
      waypoints.add(waypoint3);

      part1 = TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    {
      var waypoints = new ArrayList<Pose2d>();

      var waypoint1 = new Pose2d(3.81, 1.524, new Rotation2d());
      var waypoint2 = new Pose2d(6.096, 0.762, new Rotation2d().fromDegrees(90.0));
      var waypoint3 = new Pose2d(6.096, 2.286, new Rotation2d().fromDegrees(180.0));
      var waypoint4 = new Pose2d(2.286, 2.286, new Rotation2d().fromDegrees(180.0));
      var waypoint5 = new Pose2d(2.286, 2.286, new Rotation2d());

      waypoints.add(waypoint1);
      waypoints.add(waypoint2);
      waypoints.add(waypoint3);
      waypoints.add(waypoint4);
      waypoints.add(waypoint5);

      part2 = TrajectoryGenerator.generateTrajectory(waypoints, config);
    }
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (part1 == null || part2 == null) {
      generateTrajectories();
    }
    runCommand(new Aim());
    runCommand(new IntakeCommand(true));
    ArrayList<Command> driveAndShoot = new ArrayList<>();
    driveAndShoot.add(new ShootWhileDriving());
    driveAndShoot.add(new DriveTrajectory(part1));
    runCommand(new ParallelCommand(driveAndShoot));
    runCommand(new WaitCommand(2));
    runCommand(new StowShooter());
    runCommand(new DriveTrajectory(part2));
    runCommand(new IntakeCommand(false));
    runCommand(new Shoot(3));
    runCommand(new StowShooter());
  }
}