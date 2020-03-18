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
import frc2020.auto.FieldConstants;
import frc2020.auto.commands.DriveTrajectory;
import frc2020.auto.commands.WaitCommand;
import frc2020.robot.Constants;
import frc2020.subsystems.Drive;
import java.util.List;

public class Right13Ball extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = Drive.getInstance().getFeedforward();

    public static Trajectory rightToTrench = null;
    public static Trajectory trenchToTrench = null;
    public static Trajectory trenchToShoot = null;
    public static Trajectory shootToRendezous = null;
    public static Trajectory rendezvousToNextShoot = null;

    public static void generateTrajectories() {
        var maxVoltage = 10.0; // volts
        var maxAccel = 1.5; // meters / seconds*2
        var maxVelocity = 1.5; // meters / seconds

        // Second trench ball
        var secondTrenchBallPickup = new Pose2d(FieldConstants.SECOND_TRENCH_BALL_X - Constants.ROBOT_LENGTH*.5 - Constants.INTAKE_LENGTH,
                                                FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());
        // Pickup for trench balls 4 and 5
        var fourthTrenchBallPickup = new Pose2d(FieldConstants.FOURTH_TRENCH_BALL_X - Constants.ROBOT_LENGTH*.5 - Constants.INTAKE_LENGTH,
                                                FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());

        {
            var startPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + Constants.ROBOT_LENGTH/2,
                    FieldConstants.RIGHT_POSITION_Y, new Rotation2d());

            List<Translation2d> midPoints = List.of(
            );

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            rightToTrench = TrajectoryGenerator.generateTrajectory(
                    startPose, midPoints, secondTrenchBallPickup, config);

        }

        {
            List<Translation2d> midPoints = List.of(
            );

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            trenchToTrench = TrajectoryGenerator.generateTrajectory(
                    secondTrenchBallPickup, midPoints, fourthTrenchBallPickup, config);
        }

        {
            List<Translation2d> midPoints = List.of(
                );
    
            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);
    
            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                        .addConstraint(voltageConstraint);
            config.setReversed(true);
    
            trenchToShoot = TrajectoryGenerator.generateTrajectory(
                fourthTrenchBallPickup, midPoints, secondTrenchBallPickup, config);
        }

        {
            var midPoint1 = new Translation2d(FieldConstants.BALL_FIVE_X + Constants.INTAKE_WIDTH*.25,
                    FieldConstants.BALL_FIVE_Y + Constants.ROBOT_LENGTH/2 + Constants.INTAKE_LENGTH);
            var midPoint2 = new Translation2d(FieldConstants.BALL_FOUR_X + Constants.INTAKE_WIDTH*.25,
                    FieldConstants.BALL_FOUR_Y + Constants.ROBOT_LENGTH/2 + Constants.INTAKE_LENGTH);
            var midPoint3 = new Translation2d(FieldConstants.BALL_THREE_X + Constants.INTAKE_WIDTH*.25,
                    FieldConstants.BALL_THREE_Y + Constants.ROBOT_LENGTH/2 + Constants.INTAKE_LENGTH);
            var midPoint4 = new Translation2d(FieldConstants.BALL_TWO_X + Constants.INTAKE_WIDTH*.25,
                    FieldConstants.BALL_TWO_Y + Constants.ROBOT_LENGTH/2 + Constants.INTAKE_LENGTH);


            List<Translation2d> midPoints = List.of(
                    midPoint1,
                    midPoint2,
                    midPoint3,
                    midPoint4
            );

            var endPose = new Pose2d(FieldConstants.BALL_ONE_X + Constants.INTAKE_WIDTH*.25,
                    FieldConstants.BALL_ONE_Y + Constants.ROBOT_LENGTH/2 + Constants.INTAKE_LENGTH,
                    Rotation2d.fromDegrees(270 + FieldConstants.SHIELD_GENERATOR_ANGLE));

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            shootToRendezous = TrajectoryGenerator.generateTrajectory(
                    secondTrenchBallPickup, midPoints, endPose, config);
        }

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        if (rightToTrench == null || trenchToTrench == null || trenchToShoot == null || shootToRendezous == null) {
            generateTrajectories();
        }
        runCommand(new DriveTrajectory(rightToTrench, true));
        runCommand(new DriveTrajectory(trenchToTrench));
        runCommand(new DriveTrajectory(trenchToShoot));
        runCommand(new DriveTrajectory(shootToRendezous));

    }
}
