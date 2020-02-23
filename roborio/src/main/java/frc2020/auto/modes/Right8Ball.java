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

/**
 * Robot starts in front of trench, backs into first two trench balls, then shoots
 */
public class Right8Ball extends AutoMode {
    private static DifferentialDriveKinematics DRIVE_KINEMATICS = Drive.getInstance().getKinematics();
    private static SimpleMotorFeedforward FEEDFORWARD = Drive.getInstance().getFeedforward();

    public static Trajectory rightToTrench = null; // picks up first two trench balls
    public static Trajectory trenchToShoot1 = null; // drives backwards to shooting pose
    public static Trajectory shootToTrench = null; // picks up next three trench balls
    public static Trajectory trenchToShoot2 = null; // drives backwards to shooting pose

    public static void generateTrajectories() {
        var maxVoltage = 10.0; // volts
        var maxAccel = 1.5; // meters / seconds*2
        var maxVelocity = 1.5; // meters / seconds

        // Second trench ball
        var secondTrenchBallPickup = new Pose2d(FieldConstants.SECOND_TRENCH_BALL_X - Constants.ROBOT_LENGTH*.5 - Constants.INTAKE_LENGTH,
                                                FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());
        // Arbitrarily picked point in front of power port
        var shootPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + 1, FieldConstants.CENTER_POWER_PORT_Y, new Rotation2d());
        // Pickup for trench balls 4 and 5
        var fourthTrenchBallPickup = new Pose2d(FieldConstants.FOURTH_TRENCH_BALL_X - Constants.ROBOT_LENGTH*.5 - Constants.INTAKE_LENGTH,
        FieldConstants.THIRD_TRENCH_BALL_Y, new Rotation2d());

        {
            // Right in front of trench
            var startPose = new Pose2d(FieldConstants.ALLIANCE_WALL_TO_INITIATION_X + Constants.ROBOT_LENGTH*.5,
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
            config.setReversed(true);

            trenchToShoot1 = TrajectoryGenerator.generateTrajectory(
                    secondTrenchBallPickup, midPoints, shootPose, config);
        }

        {
            List<Translation2d> midPoints = List.of(
                secondTrenchBallPickup.getTranslation()
            );

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);

            shootToTrench = TrajectoryGenerator.generateTrajectory(
                        shootPose, midPoints, fourthTrenchBallPickup, config);

        }

        {
            List<Translation2d> midPoints = List.of(
                secondTrenchBallPickup.getTranslation()
            );

            var voltageConstraint = new DifferentialDriveVoltageConstraint(FEEDFORWARD, DRIVE_KINEMATICS, maxVoltage);

            var config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(DRIVE_KINEMATICS)
                    .addConstraint(voltageConstraint);
            config.setReversed(true);

            trenchToShoot2 = TrajectoryGenerator.generateTrajectory(
                        fourthTrenchBallPickup, midPoints, shootPose, config);  
        }

    }

    @Override
	protected void routine() throws AutoModeEndedException {
		if (rightToTrench == null || trenchToShoot1 == null || shootToTrench == null) {
            generateTrajectories();
        }

        runCommand(new DriveTrajectory(rightToTrench, true));
        runCommand(new DriveTrajectory(trenchToShoot1));
        runCommand(new DriveTrajectory(shootToTrench));
        runCommand(new DriveTrajectory(trenchToShoot2));
		
    }
}
