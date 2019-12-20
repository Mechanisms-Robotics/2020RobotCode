package frc2020.states;

import frc2020.subsystems.Drive;
import frc2020.util.DriveSignal;

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
        * Default constructor, initializes demand, type, and lowGear
        */
        public DriveDemand(DriveSignal demand) {
            this.demand = demand;
            this.type = DemandType.OpenLoop;
            this.inLowGear = false;
        }

        public static enum DemandType {
            OpenLoop,
            Velocity
        }
    }

    //Setters and getters for each subsystem demand
    public void setDriveDemand(DriveDemand demand) {
        driveDemand = demand;
    }

    public DriveDemand getDriveDemand() {
        return driveDemand;
    }

    /**
     * Tries to update all the subystems for the robot
     * from this command state
     * @param drive An istance of the drive train subystem
     */
    public void updateSubsystems(Drive drive) {
        maybeUpdateDrive(drive);
    }

    /**
     * Tries to update the drive train with a
     * commanded demand and demand type.
     * @param drive An instance of the drive train subsytem
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
