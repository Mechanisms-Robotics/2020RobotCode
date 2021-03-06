package frc2020.auto.modes;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Translation2d;
//Commented this out as this was changed from a cubic to a quintic spline
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
        var maxAccel = 2.3; // m/s
        var maxVelocity = 2.3; // m/s

        // This can be a list of Translation2d and Pose2d
        // Note if you use a Pose2d WPILib will
        // generate a quntic spline instead of a cubic
        // spline. (This may take longer)
        var waypointStart = new Pose2d(0.0, 0.0, new Rotation2d());

        var waypoint1 = new Pose2d(3.048, 0, Rotation2d.fromDegrees(-45.0));
        //var waypoint2 = new Pose2d(3.429, -.762, Rotation2d.fromDegrees(90));
        var waypoint3 = new Pose2d(3.048, -1.643, Rotation2d.fromDegrees(180));
        var waypoint4 = new Pose2d(2.157, -.762, Rotation2d.fromDegrees(90));
       // var waypoint5 = new Pose2d(3.81, 0, new Rotation2d());
        var waypoint6 = new Pose2d(4.953, 0, Rotation2d.fromDegrees(7));
        var waypoint7 = new Pose2d(6.296, .762, Rotation2d.fromDegrees(105));
        var waypoint8 = new Pose2d(5.334, 1.643, Rotation2d.fromDegrees(-170));
        //var waypoint9 = new Pose2d(4.953, .762, Rotation2d.fromDegrees(85));
        //var waypoint10 = new Pose2d(4.953, 0, Rotation2d.fromDegrees(45));
        var waypoint11 = new Pose2d(6.096, -1.143, Rotation2d.fromDegrees(-30));
        //var waypoint12 = new Pose2d(6.858, -1.243, Rotation2d.fromDegrees(30));
        var waypoint13 = new Pose2d(8.02, -.762, Rotation2d.fromDegrees(90));
        var waypoint14 = new Pose2d(6.558, 0.0, Rotation2d.fromDegrees(180));
        //var waypoint15 = new Pose2d(4.953, 0, Rotation2d.fromDegrees(-170));
        //var waypoint16 = new Pose2d(3.048, .381, Rotation2d.fromDegrees(180));

        var wayPointEnd = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));
        
        var waypoints = new ArrayList<Pose2d>();
        waypoints.add(waypointStart);
        waypoints.add(waypoint1);
        //waypoints.add(waypoint2);
        waypoints.add(waypoint3);
        waypoints.add(waypoint4);
        //waypoints.add(waypoint5);
        waypoints.add(waypoint6);
        waypoints.add(waypoint7);
        waypoints.add(waypoint8);
        //waypoints.add(waypoint9);
        //waypoints.add(waypoint10);
        waypoints.add(waypoint11);
        //waypoints.add(waypoint12);
        waypoints.add(waypoint13);
        waypoints.add(waypoint14);
        //waypoints.add(waypoint15);
        //waypoints.add(waypoint16);
        waypoints.add(wayPointEnd);
        
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
            waypoints, config);
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