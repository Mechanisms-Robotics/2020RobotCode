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
public class Basic3Ball extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = 
        Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = 
        Drive.getInstance().getFeedforward();

    private static Trajectory autoLine = null;
    
    public static void generateTrajectories() {
        var maxVoltage = 10.0; // Voltes
        var maxAccel = 1.5; // m/s
        var maxVelocity = 1.5; // m/s

        var startPose = new Pose2d();

        // This can be a list of Translation2d and Pose2d
        // Note if you use a Pose2d WPILib will
        // generate a quntic spline instead of a cubic
        // spline. (This may take longer)
        var midPoints = new ArrayList<Translation2d>();
        var endPose = new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(0.0));
        
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
		runCommand(new Shoot(5.0));
	}
}