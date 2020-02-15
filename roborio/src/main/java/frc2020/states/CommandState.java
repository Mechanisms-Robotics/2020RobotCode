package frc2020.states;

import frc2020.subsystems.Drive;
import frc2020.subsystems.Feeder;
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
    public boolean manualDemand = false;

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
    public void updateSubsystems(Drive drive, Limelight limelight, Feeder feeder) { 
        maybeUpdateLimelight(limelight);
        maybeUpdateDrive(drive, limelight);
        if (manualDemand) {
            maybeUpdateFeeder(feeder);
            feederDemand = null;
        } else { // TODO: Remove when superstructure implemented
            feederDemand = new FeederDemand();
            maybeUpdateFeeder(feeder);
        }
        driveDemand = null;
        limelightDemand = null;
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
            feeder.runFeeder(feederDemand.outtake);
        } else {
            feeder.stop();
        }
    }
}
