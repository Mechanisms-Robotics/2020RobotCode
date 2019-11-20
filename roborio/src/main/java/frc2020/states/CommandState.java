package frc2020.states;

import frc2020.subsystems.Drive;
import frc2020.util.DriveSignal;

public class CommandState {

    public DriveDemand driveDemand;
    public static class DriveDemand {
        public DriveSignal demand;
        public DemandType type;

        public DriveDemand(DriveSignal demand, DemandType type) {
            this.demand = demand;
            this.type = type;
        }

        public DriveDemand(DriveSignal demand) {
            this.demand = demand;
            this.type = DemandType.OpenLoop;
        }

        public static enum DemandType {
            OpenLoop,
            Velocity
        }
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
        }
    }
}