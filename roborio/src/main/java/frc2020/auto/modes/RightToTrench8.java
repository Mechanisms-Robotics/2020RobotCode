package frc2020.auto.modes;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.FieldConstants;
import frc2020.auto.commands.*;
import frc2020.robot.Constants;
import frc2020.subsystems.Drive;

/**
 * Starts in front of power port, shoots 3 balls, goes to trench and takes two balls,
 * turn 90 degrees to the right, and shoots those balls.
 */
public class RightToTrench8 extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = Drive.getInstance().getFeedforward();

    public static Trajectory rightToShoot = null;

    public static void generateTrajectories() {
        var maxVoltage = 10.0; // volts
        var maxAccel = 1.5; // meters / seconds*2
        var maxVelocity = 1.5; // meters / seconds
        Pose2d shootPose2d; //where bot stops to shoot balls
        {
            var startPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + Constants.ROBOT_LENGTH/2, 
             FieldConstants.RIGHT_POSITION_Y,
                    new Rotation2d());

            List<Translation2d> midPoints = List.of(
                new Translation2d(FieldConstants.THIRD_TRENCH_BALL_X - 0.9144 - Constants.ROBOT_LENGTH/2,
                 FieldConstants.THIRD_TRENCH_BALL_Y - Constants.ROBOT_WIDTH*.25)
            );

            shootPose2d = new Pose2d(FieldConstants.SECOND_TRENCH_BALL_X, FieldConstants.THIRD_TRENCH_BALL_Y - 1.0668,
             Rotation2d.fromDegrees(-90));

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            rightToShoot = TrajectoryGenerator.generateTrajectory(
                startPose, midPoints, shootPose2d, config);

        }
            
        
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        if (rightToShoot == null) {
            generateTrajectories();
        }
        runCommand(new DriveTrajectory(rightToShoot, true));
        runCommand(new WaitCommand(2.0));

    }
}