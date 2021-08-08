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
public class StealAuto32 extends AutoMode {
  private static DifferentialDriveKinematics DRIVE_KINEMATICS =
      Drive.getInstance().getKinematics();
  private static SimpleMotorFeedforward FEEDFORWARD =
      Drive.getInstance().getFeedforward();

  private static Trajectory startToSteal = null;
  private static Trajectory stealToStart = null;

  private static double lineDistance = 1.5; // meters

  public static void generateTrajectories() {
    var maxVoltage = 10.0; // Voltes
    var maxAccel = 1.0; // m/s
    var maxVelocity = 1.0; // m/s

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
    {
      var startPose = new Pose2d();

      // This can be a list of Translation2d and Pose2d
      // Note if you use a Pose2d WPILib will
      // generate a quntic spline instead of a cubic
      // spline. (This may take longer)
      var midPoints = new ArrayList<Translation2d>();
      var endPose = new Pose2d(lineDistance, 0.0, Rotation2d.fromDegrees(0.0));

      startToSteal = TrajectoryGenerator.generateTrajectory(
          startPose, midPoints, endPose, config);
    }

    {
      var startPose = new Pose2d(lineDistance, 0.0, Rotation2d.fromDegrees(0.0));

      // This can be a list of Translation2d and Pose2d
      // Note if you use a Pose2d WPILib will
      // generate a quntic spline instead of a cubic
      // spline. (This may take longer)
      var midPoints = new ArrayList<Translation2d>();
      var endPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

      stealToStart = TrajectoryGenerator.generateTrajectory(
          startPose, midPoints, endPose, config);
    }
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (startToSteal == null || stealToStart == null) {
      generateTrajectories();
    }
    runCommand(new Shoot(3.5));
    runCommand(new IntakeCommand(true));
    runCommand(new DriveTrajectory(startToSteal));
    runCommand(new WaitCommand(0.5));
    runCommand(new DriveTrajectory(stealToStart));
    runCommand(new Shoot(3.5));
  }
}