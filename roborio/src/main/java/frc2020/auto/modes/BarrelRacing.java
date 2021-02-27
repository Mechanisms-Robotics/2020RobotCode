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
public class BarrelRacing extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = 
        Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = 
        Drive.getInstance().getFeedforward();

    private static Trajectory autoLine = null;
    
    public static void generateTrajectories() {
        var maxVoltage = 10.0; // Voltes
        var maxAccel = 0.5; // m/s
        var maxVelocity = 0.5; // m/s

        var startPose = new Pose2d();

        // This can be a list of Translation2d and Pose2d
        // Note if you use a Pose2d WPILib will
        // generate a quntic spline instead of a cubic
        // spline. (This may take longer)
        // Don't forget to input the degree orientation of the robot

        var midPoint1 = new Translation2d(3.048, 0.0);
        var midPoint2 = new Translation2d(3.429, -.762);
        var midPoint3 = new Translation2d(3.048, -1.143);
        var midPoint4 = new Translation2d(2.667, -.762);
        var midPoint5 = new Translation2d(3.81, 0.0);
        var midPoint6 = new Translation2d(4.953, 0.0);
        var midPoint7 = new Translation2d(6.096, .762);
        var midPoint8 = new Translation2d(5.334, 1.143);
        var midPoint9 = new Translation2d(4.953, .762);
        var midPoint10 = new Translation2d(4.953, 0.0);
        var midPoint11 = new Translation2d(6.096, -1.143);
        var midPoint12 = new Translation2d(6.858, -1.243);
        var midPoint13 = new Translation2d(7.62, -.762);
        var midPoint14 = new Translation2d(6.858, 0.0);
        var midPoint15 = new Translation2d(4.953, 0.0);
        var midPoint16 = new Translation2d(3.048, .381);

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
        midPoints.add(midPoint11);
        midPoints.add(midPoint12);
        midPoints.add(midPoint13);
        midPoints.add(midPoint14);
        midPoints.add(midPoint15);
        midPoints.add(midPoint16);

        var endPose = new Pose2d(0.0, 1.524, Rotation2d.fromDegrees(0.0));
        
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

        autoLine = TrajectoryGenerator.generateTrajectory(
            startPose, midPoints, endPose, config);
    }

    @Override
	protected void routine() throws AutoModeEndedException {
        if (autoLine == null) {
            generateTrajectories();
        }
        runCommand(new DriveTrajectory(autoLine));
		//runCommand(new Shoot(5.0));
	}
}