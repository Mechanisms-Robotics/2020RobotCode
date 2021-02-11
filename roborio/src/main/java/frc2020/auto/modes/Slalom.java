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
    var maxAccel = 0.7; // m/s
    var maxVelocity = 0.7; // m/s

    var startPose = new Pose2d();

    // This can be a list of Translation2d and Pose2d
    // Note if you use a Pose2d WPILib will
    // generate a quntic spline instead of a cubic
    // spline. (This may take longer)
    var midPoint1 = new Translation2d(1.52, 0.76);
    var midPoint2 = new Translation2d(2.29, 1.52);
    var midPoint3 = new Translation2d(3.81, 1.83);
    var midPoint4 = new Translation2d(4.11, 2.13);
    var midPoint5 = new Translation2d(5.79, 2.44);
    var midPoint6 = new Translation2d(6.1 , 0.3);

    var midPoints = new ArrayList<Translation2d>();
    midPoints.add(midPoint1);
    midPoints.add(midPoint2);
    midPoints.add(midPoint3);
    midPoints.add(midPoint4);
    midPoints.add(midPoint5);
    midPoints.add(midPoint6);
    var endPose = new Pose2d(0.0, 1.52, Rotation2d.fromDegrees(180.0));

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
        startPose, midPoints, endPose, config);
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (slalomTrajectory == null) {
      generateTrajectories();
    }
    runCommand(new DriveTrajectory(slalomTrajectory));
  }
}