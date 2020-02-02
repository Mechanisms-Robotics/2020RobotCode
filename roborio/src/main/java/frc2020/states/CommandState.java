package frc2020.states;

import frc2020.subsystems.Drive;
import frc2020.util.DriveSignal;

/**
 * Handles the general command state of the robot. If there are any subsystem commands
 * that send demands they come through here
 */
public class CommandState {
    //Demand variables, these are modified in the CSGenerators
    public DriveDemand driveDemand;

    //Subsystem demands are defined as mini classes
    public static class DriveDemand {
        public DriveSignal demand;
        public DemandType type;
        public boolean inLowGear;

        /**
        * Default constructor, initializes demand, type, and lowGear
        */
        public DriveDemand(DriveSignal demand, DemandType type, boolean lowGear) {
            this.demand = demand;
            this.type = type;
            this.inLowGear = type.equals(DemandType.OpenLoop) ? lowGear : false;
        }

        /**
        * Default constructor, initializes demand, type, and lowGear
        */
        public DriveDemand(DriveSignal demand, DemandType type) {
            this.demand = demand;
            this.type = type;
            this.inLowGear = false;
        }

        /**
        * Default constructor, initializes demand, OpenLoop, and lowGear
        */
        public DriveDemand(DriveSignal demand) {
            this.demand = demand;
            this.type = DemandType.OpenLoop;
            this.inLowGear = false;
        }

        /**
         * All possible demand types are inserted here
         */
        public static enum DemandType {
            OpenLoop,
            Velocity
        }

        /**
         * Previously used for Demand Type conversions
         * @param dsType numeric value
         * @return demand type
         */
        public static DemandType fromDSType(int dsType) {
            if (dsType == 0) {
                return DemandType.OpenLoop;
            } else {
                return DemandType.Velocity;
            }
        }
    }

    /**
     * Setter for each subsystem demand
     * @param demand
     */
    public void setDriveDemand(DriveDemand demand) {
        driveDemand = demand;
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
    public void updateSubsystems(Drive drive) {
        maybeUpdateDrive(drive);
    }

    /**
     * Tries to update the drive train with a
     * commanded demand and demand type.
     * @param drive An instance of the drive train subsystem
     */
    private void maybeUpdateDrive(Drive drive) {
        if (driveDemand != null) {
            if (driveDemand.type == DriveDemand.DemandType.Velocity) {
                drive.driveVelocity(driveDemand.demand);
            } else {
                drive.openLoop(driveDemand.demand);
            }
            if (driveDemand.inLowGear) {
                drive.setLowGear();
            } else {
                drive.setHighGear();
            }
        }
    }
}
