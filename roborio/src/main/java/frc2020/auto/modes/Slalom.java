package frc2020.auto.modes;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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

/**
 * Test auto mode for just trying out different paths, velocities, accelerations, etc
 */
public class Slalom extends AutoMode {
  private static DifferentialDriveKinematics DRIVE_KINEMATICS =
      Drive.getInstance().getKinematics();
  private static SimpleMotorFeedforward FEEDFORWARD =
      Drive.getInstance().getFeedforward();

  private static Trajectory slalomTrajectory = null;

  public static void generateTrajectories() {
    var maxVoltage = 10.0; // Voltes
    var maxAccel = 1.5; // m/s
    var maxVelocity = 1.5; // m/s

    var waypoints = new ArrayList<Pose2d>();
    var waypoint1 = new Pose2d(0.0, 0.0, new Rotation2d());
    var waypoint2 = new Pose2d(1.163, 0.762, Rotation2d.fromDegrees(45.0));
    var waypoint3 = new Pose2d(3.429, 1.524, new Rotation2d());
    var waypoint4 = new Pose2d(6.150, 0.762, Rotation2d.fromDegrees(-60.0));
    var waypoint5 = new Pose2d(7.989, 0.962, Rotation2d.fromDegrees(90.0));
    var waypoint6 = new Pose2d(6.625, 0.762, Rotation2d.fromDegrees(-110.0));
    var waypoint7 = new Pose2d(3.429, 0.0, Rotation2d.fromDegrees(-180.0));
    var waypoint8 = new Pose2d(1.824, 0.762, Rotation2d.fromDegrees(105.0));
    var waypoint9 = new Pose2d(0.0, 1.524, Rotation2d.fromDegrees(180.0));

    waypoints.add(waypoint1);
    waypoints.add(waypoint2);
    waypoints.add(waypoint3);
    waypoints.add(waypoint4);
    waypoints.add(waypoint5);
    waypoints.add(waypoint6);
    waypoints.add(waypoint7);
    waypoints.add(waypoint8);
    waypoints.add(waypoint9);

    // Define other constraintes
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

    slalomTrajectory = TrajectoryGenerator.generateTrajectory(
        waypoints, config);
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (slalomTrajectory == null) {
      generateTrajectories();
    }
    runCommand(new DriveTrajectory(slalomTrajectory));
  }
}