package frc2020.auto.modes;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.commands.DriveTrajectory;
import frc2020.auto.commands.IntakeCommand;
import frc2020.auto.commands.Shoot;
import frc2020.subsystems.Drive;

import java.util.ArrayList;

/**
 * Test auto mode for just trying out different paths, velocities, accelerations, etc
 */
public class IntakeTestingAuto extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = 
        Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = 
        Drive.getInstance().getFeedforward();

    private static Trajectory traj = null;
    
    public static void generateTrajectories() {
        var maxVoltage = 10.0; // Volts
        var maxAccel = 1.5; // m/s
        var maxVelocity = 1.5; // m/s
        var distance = 2.0; //m

        var startPose = new Pose2d();
        var midPoints = new ArrayList<Translation2d>();
        var endPose = new Pose2d(distance, 0.0, Rotation2d.fromDegrees(0.0));

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

        traj = TrajectoryGenerator.generateTrajectory(
            startPose, midPoints, endPose, config);
    }

    @Override
	protected void routine() throws AutoModeEndedException {
        if (traj == null) {
            generateTrajectories();
        }
        runCommand(new IntakeCommand(true));
        runCommand(new DriveTrajectory(traj));
	}
}