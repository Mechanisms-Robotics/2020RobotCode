package frc2020.states;

import frc2020.subsystems.*;
import frc2020.util.Logger;
import frc2020.subsystems.Climber;
import frc2020.subsystems.ControlPanel;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Feeder;
import frc2020.subsystems.Flywheel;
import frc2020.subsystems.Intake;
import frc2020.subsystems.Limelight;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.util.DriveSignal;

/**
 * Handles the general command state of the robot. If there are any subsystem commands
 * that send demands they come through here
 */
public class CommandState {
    //Demand variables, these are modified in the CSGenerators
    public DriveDemand driveDemand;
    public LimelightDemand limelightDemand;
    public FeederDemand feederDemand;
    public IntakeDemand intakeDemand;
    public FlywheelDemand flywheelDemand;
    public ControlPanelDemand controlPanelDemand;
    public ClimberDemand climberDemand;
    public boolean manualDemand = false;
    private TurretDemand turretDemand;
    private HoodDemand hoodDemand;
    private ShooterDemand shooterDemand;

    private Logger logger_ = Logger.getInstance();

    //Subsystem demands are defined as mini classes
    public static class DriveDemand {
        public DriveSignal signal = null;
        public DemandType type = DemandType.OpenLoop;
        public boolean inLowGear = false;
        public boolean autoSteer = false;
        public boolean autoBackup = false;

        /**
         * All possible demand types are inserted here
         */
        public enum DemandType {
            OpenLoop,
            Velocity
        }

        public static DriveDemand autoSteer(DriveSignal signal) {
            DriveDemand demand = new DriveDemand();
            demand.signal = signal;
            demand.autoSteer = true;
            demand.autoBackup = false;
            return demand;
        }

        public static DriveDemand autoBackup() {
            DriveDemand demand = new DriveDemand();
            demand.autoSteer = false;
            demand.autoBackup = true;
            return demand;
        }

        public static DriveDemand fromSignal(DriveSignal signal, boolean lowGear) {
            DriveDemand demand = new DriveDemand();
            demand.signal = signal;
            demand.inLowGear = lowGear;
            return demand;
        }
    }

    public static class LimelightDemand {
        public Limelight.LedMode ledMode = Limelight.LedMode.PIPELINE;
        public int pipeline = 0;
    }

    public static class FeederDemand {
        public boolean outtake = false;
        public boolean intake = false;
    }

    public static class IntakeDemand {
        public boolean outtake = false;
        public boolean intake = false;
        public boolean deploy = false;
    }

    public static class FlywheelDemand {
        public boolean spin = false;
    }

    public static class ControlPanelDemand {
        public boolean clockwise = false;
        public boolean counterclockwise = false;
        public boolean deploy = false;
        public boolean rotation = false;
        public boolean position = false;
    }

    public static class ClimberDemand {
        public boolean deploy = false;
        public boolean lock = false;
        public double rightWinchSpeed = 0.0;
        public double leftWinchSpeed = 0.0;
    }

    public static class TurretDemand {
        public double speed = 0.0;
        public boolean useOpenLoop = true;
    }

    public static class HoodDemand {
        public double speed = 0.0;
        public boolean deploy = false;
    }

    public static class ShooterDemand {
        public Shooter.ShooterState state = Shooter.ShooterState.Stowed;
        public boolean overrideFeeder = false;
    }

    public void setManualControl(boolean manualControl) {
        manualDemand = manualControl;
    }

    /**
     * Setter for each subsystem demand
     * @param demand
     */
    public void setDriveDemand(DriveDemand demand) {
        driveDemand = demand;
    }

    public void setLimelightDemand(LimelightDemand demand) {
        limelightDemand = demand;
    }

    public void setFeederDemand(FeederDemand demand) {
        feederDemand = demand;
    }

    public void setIntakeDemand(IntakeDemand demand) {
        intakeDemand = demand;
    }

    public void setFlywheelDemand(FlywheelDemand demand) {
        flywheelDemand = demand;
    }

    public void setControlPanelDemand(ControlPanelDemand demand) {
        controlPanelDemand = demand;
    }

    public void setClimberDemand(ClimberDemand demand) {
        climberDemand = demand;
    }

    public void setTurretDemand(TurretDemand demand) {
        turretDemand = demand;
    }

    public void setHoodDemand(HoodDemand demand) {
        hoodDemand = demand;
    }

    public void setShooterDemand(ShooterDemand demand) { shooterDemand = demand; }
    /**
     * Getter for each subsystem demand
     * @return
     */
    public DriveDemand getDriveDemand() {
        return driveDemand;
    }

    /**
     * Tries to update all the subsystems for the robot
     * from this command state
     * @param drive An instance of the drive train subsystem
     */
    public void updateSubsystems(Drive drive, Limelight limelight, Feeder feeder, Turret turret,
                                 Intake intake, Flywheel flywheel, Climber climber, Hood hood, Shooter shooter, ControlPanel controlPanel) {
        maybeUpdateLimelight(limelight);
        maybeUpdateDrive(drive, limelight);
        maybeUpdateIntake(intake);
        maybeUpdateClimber(climber);
        maybeUpdateControlPanel(controlPanel);
        if (shooterDemand.overrideFeeder || shooter.getWantedState() == Shooter.ShooterState.Manual) {
            maybeUpdateFeeder(feeder);
        }
        maybeUpdateShooter(shooter);
        if (shooter.getWantedState() == Shooter.ShooterState.Manual) {
            maybeUpdateTurret(turret);
            maybeUpdateHood(hood);
            maybeUpdateFlywheel(flywheel);
        }
    }

    /**
     * Tries to update the drive train with a
     * commanded demand and demand type.
     * @param drive An instance of the drive train subsystem
     */
    private void maybeUpdateDrive(Drive drive, Limelight limelight) {
        if (driveDemand != null) {
            if (driveDemand.autoSteer) {
                double average = (driveDemand.signal.getLeft() + driveDemand.signal.getRight()) / 2.0;
                drive.autoSteer(limelight.getTargetReading().azimuth, average);
            } else if (driveDemand.autoBackup) {
                drive.autoBackup();
            } else if (driveDemand.type == DriveDemand.DemandType.Velocity) {
                drive.driveVelocity(driveDemand.signal);
            } else {
                drive.openLoop(driveDemand.signal);
            }
            if (driveDemand.inLowGear) {
                drive.setLowGear();
            } else {
                drive.setHighGear();
            }
        }
        driveDemand = null;
    }

    /**
     * Tries to update the limelight with a
     * commanded demand type.
     * @param limelight The limelight to update
     */
    private void maybeUpdateLimelight(Limelight limelight) {
        if (limelightDemand != null) {
            limelight.setPipeline(limelightDemand.pipeline);
            limelight.setLed(limelightDemand.ledMode);
            limelightDemand = null;
        }
    }

    private void maybeUpdateFeeder(Feeder feeder) {
        if (feederDemand != null) {
            if (feederDemand.outtake && feederDemand.intake) {
                logger_.logInfo("Both feeder intake and outake hats pressed");
                feeder.stop();
            } else if (feederDemand.outtake) {
                feeder.runFeeder(true);
            } else if (feederDemand.intake) {
                feeder.runFeeder(false);
            } else {
                feeder.stop();
            }
            feederDemand = null;
        }
    }

    private void maybeUpdateIntake(Intake intake) {
        if (intakeDemand != null) {
            if (intakeDemand.outtake && intakeDemand.intake) {
                logger_.logInfo("Both intake intake and outake buttons pressed");
                intake.stop();
            } else if (intakeDemand.outtake) {
                intake.runIntake(true);
            } else if (intakeDemand.intake) {
                intake.runIntake(false);
            } else {
                intake.stop();
            }

            if (intakeDemand.deploy) {
                intake.deployIntake();
            } else {
                intake.stowIntake();
            }
            intakeDemand = null;
        }
    }

    private void maybeUpdateFlywheel(Flywheel flywheel) {
        if (flywheelDemand != null) {
            if (flywheelDemand.spin) {
                flywheel.spinFlywheel();
            } else {
                flywheel.setOpenLoop(0.0);
            }
            flywheelDemand = null;
        }
    }

    private void maybeUpdateControlPanel(ControlPanel controlPanel) {
        if (controlPanelDemand != null) {
            if (manualDemand) {
                controlPanel.startManualControl();
                if (controlPanelDemand.clockwise && controlPanelDemand.counterclockwise) {
                    logger_.logInfo("Both control panel forward and reverse buttons pressed");
                    controlPanel.stop();
                } else if (controlPanelDemand.clockwise) {
                    controlPanel.runPanelWheel(true);
                } else if (controlPanelDemand.counterclockwise) {
                    controlPanel.runPanelWheel(false);
                } else {
                    controlPanel.stop();
                }
            } else {
                if (controlPanelDemand.rotation) {
                    controlPanel.toggleRotationControl();
                } else if (controlPanelDemand.position) {
                    controlPanel.togglePositionControl();
                }
            }

            if(controlPanelDemand.deploy) {
                controlPanel.deployPanelArm();
            } else {
                controlPanel.stowPanelArm();
            }
            controlPanelDemand = null;
        }
    }

    private void maybeUpdateClimber(Climber climber) {
        if (climberDemand != null) {
            climber.controlWinch(new DriveSignal(climberDemand.leftWinchSpeed, climberDemand.rightWinchSpeed, true));
            //Note that the winch demand is set first
            //If we have not yet stowed, super.stop() will override demand

            if (climberDemand.deploy) {
                climber.deployClimberArm();
            } else {
                climber.stowClimberArm();
            }

            if (climberDemand.lock) {
                climber.lockWinch();
            } else {
                climber.unlockWinch();
            }
        }
        climberDemand = null;
    }

    private void maybeUpdateTurret(Turret turret) {
        if (turretDemand != null) {
            if (turretDemand.useOpenLoop) {
                turret.setOpenLoop(turretDemand.speed);
            } else {
                turret.setRelativeRotation(new Rotation2d(turretDemand.speed));
            }
            turretDemand = null;
        }
    }

    private void maybeUpdateHood(Hood hood) {
        if (hoodDemand != null) {
            if (hoodDemand.deploy) {
                hood.deployHood();
                hood.setOpenLoop(hoodDemand.speed);
            } else {
                hood.stowHood();
            }
            hoodDemand = null;
        }
    }

    private void maybeUpdateShooter(Shooter shooter) {
        if (shooterDemand != null) {
            shooter.setState(shooterDemand.state);
            shooter.setOverrideFeeder(shooterDemand.overrideFeeder);

            shooterDemand = null;
        }
    }
}
