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
    // generate a quintic spline instead of a cubic
    // spline. (This may take longer)
    var midPoint1 = new Translation2d(1.27, 0.2032);
    var midPoint2 = new Translation2d(2.286, 1.651);
    var midPoint3 = new Translation2d(3.81, 2.032);
    var midPoint4 = new Translation2d(5.588, 1.524);
    var midPoint5 = new Translation2d(6.858, 0.2032);
    var midPoint6 = new Translation2d(8.001 , -0.762);
    var midPoint7 = new Translation2d(6.858 , 1.651);
    var midPoint8 = new Translation2d(5.588 , 0.2032);
    var midPoint9 = new Translation2d(3.81 , 0);
    var midPoint10 = new Translation2d(2.286 , 0.2032);

    var midPoints = new ArrayList<Translation2d>();
    midPoints.add(midPoint1);
    midPoints.add(midPoint2);
    midPoints.add(midPoint3);
    midPoints.add(midPoint4);
    midPoints.add(midPoint5);
    midPoints.add(midPoint6);
    midPoints.add(midPoint7);
    midPoints.add(midPoint8);
    midPoints.add(midPoint9);
    midPoints.add(midPoint10);
    var endPose = new Pose2d(0.0, 1.524, Rotation2d.fromDegrees(180.0));

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