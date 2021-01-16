package frc2020.auto.modes;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
import frc2020.auto.FieldConstants;
import frc2020.auto.commands.DriveTrajectory;
import frc2020.auto.commands.WaitCommand;
import frc2020.robot.Constants;
import frc2020.subsystems.Drive;

/**
 * Starting from intersection between initiation line and midfield line. Shoots 3 balls,
 * goes over rendevou zone picking up 5 balls and goes to trench, shoots 5 balls, backs
 * up trench collecting 5 more balls, goes back forward and shoots and 5 more balls, totaling
 * to 13 balls.
 */
public class Basic13Ball extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = Drive.getInstance().getFeedforward();

    public static Trajectory midToRendezvous = null;
    public static Trajectory rendezvousToTrench = null;
    public static Trajectory trenchToShoot = null;

    public static void generateTrajectories() {
        var maxVoltage = 10.0; // volts
        var maxAccel = 2.0; // m/s^2
        var maxVelocity = 2.0; // m/s

        // Arbitrary shooting position past rendezvous
        Pose2d rendezvousPose = new Pose2d(FieldConstants.BALL_FIVE_X - 0.1524, FieldConstants.CENTER_POWER_PORT_Y,
        Rotation2d.fromDegrees(45));
        // Pickup for balls 4 and 5
        Pose2d trenchPose = new Pose2d(FieldConstants.FOURTH_TRENCH_BALL_X - Constants.ROBOT_LENGTH/2 - Constants.INTAKE_LENGTH,
        FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());
        // Second trench ball
        var secondTrenchBallPickup = new Pose2d(FieldConstants.SECOND_TRENCH_BALL_X - Constants.ROBOT_LENGTH*.5 - Constants.INTAKE_LENGTH,
                                                FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());
        {
            var startPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + .5 * Constants.ROBOT_LENGTH,
                    FieldConstants.BALL_ONE_Y, new Rotation2d());

            var midPoint1 = new Translation2d(FieldConstants.BALL_TWO_X - 0.6096, FieldConstants.BALL_ONE_Y - 0.6096);
            var midPoint2 = FieldConstants.BALL_TWO_POSE.getTranslation();

            List<Translation2d> midPoints = List.of(
                midPoint1,
                midPoint2
                );

            var endPose = rendezvousPose; 
            

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            midToRendezvous = TrajectoryGenerator.generateTrajectory(startPose, midPoints, endPose, config);
        }

        {
            var startPose = rendezvousPose;

            var midPoint1 = new Translation2d(FieldConstants.FIRST_TRENCH_BALL_X + Constants.ROBOT_WIDTH/4,
                FieldConstants.THIRD_TRENCH_BALL_Y - Constants.ROBOT_LENGTH/2);

            List<Translation2d> midPoints = List.of(
                midPoint1
            );

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                        .addConstraint(voltageConstraint);
    
            rendezvousToTrench = TrajectoryGenerator.generateTrajectory(startPose, midPoints, trenchPose, config);
        }

        {
            List<Translation2d> midPoints = List.of(
            );

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                        .addConstraint(voltageConstraint);
            config.setReversed(true);
    
            trenchToShoot = TrajectoryGenerator.generateTrajectory(trenchPose, midPoints, secondTrenchBallPickup, config);
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // TODO Auto-generated method stub
        if (midToRendezvous == null || rendezvousToTrench == null || trenchToShoot == null) {
            generateTrajectories();
        }

        runCommand(new DriveTrajectory(midToRendezvous, true));
        runCommand(new WaitCommand(2.0));
        runCommand(new DriveTrajectory(rendezvousToTrench));
        runCommand(new DriveTrajectory(trenchToShoot));

    }
}