package frc2020.states;

import frc2020.states.CommandState.DriveDemand;
import frc2020.subsystems.Jetson;
import frc2020.subsystems.Jetson.JetsonStatus;
import frc2020.util.DriveSignal;
import frc2020.util.Units;

public class AutoCSGenerator implements CommandStateGenerator {

    private Jetson jetson_;
    
    public AutoCSGenerator(Jetson jetson){
        this.jetson_ = jetson;
    }

    @Override
    public CommandState getCommandState() {
        CommandState state = new CommandState();
        state.setDriveDemand(generateDriveDemand());
        return state;
    }

    private DriveDemand generateDriveDemand() {
        JetsonStatus status = jetson_.getJetsonStatus();
        double left, right;
        if (status.driveDemand.type == CommandState.DriveDemand.DemandType.Velocity) {
            left = Units.metersPerSecondToEncTicksPer100Ms(status.driveDemand.demand.getLeft());
            right = Units.metersPerSecondToEncTicksPer100Ms(status.driveDemand.demand.getRight());
        } else {
            left = 0.0;
            right = 0.0;
            System.out.println("WARNING: Received Open Loop in Auto");
        }
        return new DriveDemand(new DriveSignal(left, right), DriveDemand.DemandType.Velocity);
    }

}