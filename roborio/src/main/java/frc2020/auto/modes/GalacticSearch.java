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
import frc2020.robot.Constants;
import frc2020.util.Util;
import frc2020.subsystems.Limelight;

import java.util.ArrayList;

public class GalacticSearch extends AutoMode {

    private static DifferentialDriveKinematics DRIVE_KINEMATICS =
      Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD =
      Drive.getInstance().getFeedforward();

    private static Limelight limelight_ = null;

    private static Trajectory redPathA = null;
    private static Trajectory redPathB = null;
    private static Trajectory bluePathA = null;
    private static Trajectory bluePathB = null;
    private static Trajectory goForward = null;

    public static void generateTrajectories(Limelight lowLimelight) {
        limelight_ = lowLimelight;
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        var maxVoltage = 10.0; // Voltes
        var maxAccel = 1.5; // m/s
        var maxVelocity = 1.5; // m/s
        
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
            var point1 = new Pose2d();

            var point2 = new Pose2d(1.524, -0.762, Rotation2d.fromDegrees(-30.0));
            var point3 = new Pose2d(3.048, -1.524, Rotation2d.fromDegrees(-30.0));
            var point4 = new Pose2d(3.81, 0.6, new Rotation2d());
            var point5 = new Pose2d(8.02, 0.6, new Rotation2d());

            waypoints.add(point1);
            waypoints.add(point2);
            waypoints.add(point3);
            waypoints.add(point4);
            waypoints.add(point5);

            redPathA = TrajectoryGenerator.generateTrajectory(waypoints, config);
            waypoints.clear();
        }

        {
            var point1 = new Pose2d();

            var point2 = new Pose2d(1.524, 0.0, new Rotation2d());
            var point3 = new Pose2d(3.048, -1.524, Rotation2d.fromDegrees(-45.0));
            var point4 = new Pose2d(4.572, 0.0, new Rotation2d());
            var point5 = new Pose2d(8.02, 0.0, new Rotation2d());

            waypoints.add(point1);
            waypoints.add(point2);
            waypoints.add(point3);
            waypoints.add(point4);
            waypoints.add(point5);

            redPathB = TrajectoryGenerator.generateTrajectory(waypoints, config);
            waypoints.clear();
        }

        {
            var point1 = new Pose2d();

            var point2 = new Pose2d(2.286, 0.0, new Rotation2d());
            
            waypoints.add(point1);
            waypoints.add(point2);

            goForward = TrajectoryGenerator.generateTrajectory(waypoints, config);
            waypoints.clear();
        }

        {
            var point1 = new Pose2d(2.286, 0.0, new Rotation2d());
        }

        
    }

    @Override
	protected void routine() throws AutoModeEndedException {
        limelight_.setPipeline(Constants.POWER_CELL_PIPELINE);
        runCommand(new IntakeCommand(true));
        runCommand(new WaitCommand(0.3));
        if (limelight_.getRawData().area > 0.1) {
            if (Util.epsilonEquals(limelight_.getTargetReading().azimuth, 0.0, 5.0)) {
                runCommand(new DriveTrajectory(redPathB));
            } else {
                runCommand(new DriveTrajectory(redPathA));
            }
        } else {
            runCommand(new DriveTrajectory(goForward));
            if (limelight_.getTargetReading().azimuth < 0) {
                logger_.logDebug("Running Path B");
                //runCommand(new DriveTrajectory(bluePathB));
            } else {
                logger_.logDebug("Running Path A");
                //runCommand(new DriveTrajectory(bluePathA));
            }
        }
        runCommand(new IntakeCommand(false));
	}
}