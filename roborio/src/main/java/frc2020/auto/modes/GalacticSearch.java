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
import frc2020.subsystems.Limelight.LimelightRawData;

import java.util.ArrayList;

public class GalacticSearch extends AutoMode {

    private static DifferentialDriveKinematics DRIVE_KINEMATICS =
      Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD =
      Drive.getInstance().getFeedforward();

    private static Limelight limelight_ = null;
    private static LimelightRawData data_ = null;

    private static Trajectory redPathA = null;
    private static Trajectory redPathB = null;
    private static Trajectory bluePathA = null;
    private static Trajectory bluePathB = null;

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

            var point2 = new Pose2d(1.524, -0.381, Rotation2d.fromDegrees(-30.0));
            var point3 = new Pose2d(3.048, -1.143, Rotation2d.fromDegrees(-30.0));
            var point4 = new Pose2d(3.81, 0.881, new Rotation2d());
            var point5 = new Pose2d(8.02, 0.981, new Rotation2d());

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

            var point2 = new Pose2d(1.524, 0.381, new Rotation2d());
            var point3 = new Pose2d(3.048, -1.143, Rotation2d.fromDegrees(-45.0));
            var point4 = new Pose2d(4.572, 0.2, new Rotation2d());
            var point5 = new Pose2d(8.02, 0.2, new Rotation2d());

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

            var point2 = new Pose2d(3.81, -0.381, Rotation2d.fromDegrees(-10.0));
            var point3 = new Pose2d(4.802, 1.905, Rotation2d.fromDegrees(71.0));
            var point4 = new Pose2d(6.096, 1.343, new Rotation2d());
            var point5 = new Pose2d(8.02, 1.343, new Rotation2d());

            waypoints.add(point1);
            waypoints.add(point2);
            waypoints.add(point3);
            waypoints.add(point4);
            waypoints.add(point5);

            bluePathA = TrajectoryGenerator.generateTrajectory(waypoints, config);
            waypoints.clear();
        }

        {
            var point1 = new Pose2d();

            var point2 = new Pose2d(3.81, 0.381, Rotation2d.fromDegrees(10.0));
            var point3 = new Pose2d(5.534, 1.705, Rotation2d.fromDegrees(35.0));
            var point4 = new Pose2d(7.258, 0.681, new Rotation2d());
            var point5 = new Pose2d(8.001, 0.681, new Rotation2d());

            waypoints.add(point1);
            waypoints.add(point2);
            waypoints.add(point3);
            waypoints.add(point4);
            waypoints.add(point5);

            bluePathB = TrajectoryGenerator.generateTrajectory(waypoints, config);
            waypoints.clear();
        }

        
    }

    @Override
	protected void routine() throws AutoModeEndedException {
        limelight_.setPipeline(Constants.POWER_CELL_PIPELINE);
        runCommand(new IntakeCommand(true));

        /**
         * At the end of this the limelight should have a good set of data.
         */
        data_ = limelight_.getRawData();
        while (!data_.hasTarget || (data_.hasTarget && data_.area <= 0.1 && data_.xOffset < -10.0)) {
            data_ = limelight_.getRawData();
        }

        /**
         * If the area of the target is less than 0.1, then target is too far away so the robot must
         * run the blue paths. If the area is greater than 0.1, then the power cells are close so the
         * robot should run the red paths.
         */
        if (data_.area > 0.1) {

            /**
             * The robot starts in the same starting position and orientation for the red paths.
             * If the ball is straight ahead then the robot runs Red Path B; if else, it should
             * run Red Path A.
             */
            if (data_.xOffset < 0.0) {
                logger_.logDebug("//////////////////////////////////////////RED PATH B");
                runCommand(new DriveTrajectory(redPathB));
            } else {
                logger_.logDebug("//////////////////////////////////////////RED PATH A");
                runCommand(new DriveTrajectory(redPathA));
            }
        } else {

            /**
             * If the target is to the left of the robot, then the robot runs Blue Path B;
             * if else, it should run Blue Path A.
             */
            if (data_.xOffset < 0 && data_.xOffset > -10.0) {
                logger_.logDebug("//////////////////////////////////////////BLUE PATH B");
                runCommand(new DriveTrajectory(bluePathB));
            } else {
                logger_.logDebug("//////////////////////////////////////////BLUE PATH A");
                runCommand(new DriveTrajectory(bluePathA));
            }
        }
        runCommand(new IntakeCommand(false));
	}
}