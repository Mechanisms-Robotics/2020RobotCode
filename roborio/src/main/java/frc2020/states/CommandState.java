package frc2020.states;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.util.Logger;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Feeder;
import frc2020.subsystems.Flywheel;
import frc2020.subsystems.Intake;
import frc2020.subsystems.Limelight;
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
    public boolean manualDemand = false;

    private Logger logger_ = Logger.getInstance();

    //Subsystem demands are defined as mini classes
    public static class DriveDemand {
        public DriveSignal signal = null;
        public DemandType type = DemandType.OpenLoop;
        public boolean inLowGear = false;
        public boolean autoSteer = false;

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
        public boolean longRange = false;
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

    /**
     * Getter for each subsystem demand
     * @return
     */
    public DriveDemand getDriveDemand() {
        return driveDemand;
    }

    public void updateSubsystems(Drive drive, Limelight limelight, Feeder feeder, Intake intake) {
        maybeUpdateLimelight(limelight);
        maybeUpdateDrive(drive, limelight);
        maybeUpdateIntake(intake);
        if (manualDemand) {
            maybeUpdateFeeder(feeder);
            feederDemand = null;
        } else { // TODO: Remove when superstructure implemented
            feederDemand = new FeederDemand();
            maybeUpdateFeeder(feeder);
        }
        driveDemand = null;
        limelightDemand = null;
        intakeDemand = null;
    }

    /**
     * Tries to update all the subsystems for the robot
     * from this command state
     * @param drive An instance of the drive train subsystem
     */
    public void updateSubsystems(Drive drive, Limelight limelight, Feeder feeder, Intake intake, Flywheel flywheel) { 
        maybeUpdateLimelight(limelight);
        maybeUpdateDrive(drive, limelight);
        maybeUpdateIntake(intake);
        if (manualDemand) {
            maybeUpdateFeeder(feeder);
            maybeUpdateFlywheel(flywheel);
            feederDemand = null;
            flywheelDemand = null;
        } else { // TODO: Remove when superstructure implemented
            feederDemand = new FeederDemand();
            flywheelDemand = new FlywheelDemand();
            maybeUpdateFeeder(feeder);
            maybeUpdateFlywheel(flywheel);
        }
        driveDemand = null;
        limelightDemand = null;
        intakeDemand = null;
    }

    /**
     * Tries to update the drive train with a
     * commanded demand and demand type.
     * @param drive An instance of the drive train subsystem
     */
    private void maybeUpdateDrive(Drive drive, Limelight limelight) {
        if (driveDemand != null) {
            if ( driveDemand.autoSteer) {
                double average = (driveDemand.signal.getLeft() + driveDemand.signal.getRight()) / 2.0;
                drive.autoSteer(limelight.getTargetReading().azimuth, average);
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
        }
    }

    private void maybeUpdateIntake(Intake intake) {
        if(intakeDemand != null) {
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

            if(intakeDemand.deploy) {
                intake.deployIntake();
            } else {
                intake.stowIntake();
            }
        }
    }

    private void maybeUpdateFlywheel(Flywheel flywheel) {
        if(flywheelDemand != null) {
            if(flywheelDemand.spin) {
                if(flywheelDemand.longRange) {
                    flywheel.spinLongRangeFlywheel();
                } else {
                    flywheel.spinFlywheel();
                }
            } else {
                flywheel.stop();
            }
        }
    }
}
