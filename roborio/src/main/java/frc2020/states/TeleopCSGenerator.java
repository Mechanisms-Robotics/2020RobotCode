package frc2020.states;

import edu.wpi.first.wpilibj.Joystick;
import frc2020.robot.Constants;
import frc2020.states.CommandState.*;
import frc2020.util.*;

public class TeleopCSGenerator implements CommandStateGenerator {
    private Joystick leftJoystick_;
    private Joystick rightJoystick_;
    private LatchedBoolean driveShiftLatch;
    private boolean driveLowGear = false;

    //All ports and constants should be applied in here.
    //Anything specific to this generator should be constructed here.
    public TeleopCSGenerator(int lJoyPort, int rJoyPort) {
        leftJoystick_ = new Joystick(lJoyPort);
        rightJoystick_ = new Joystick(rJoyPort);
        driveShiftLatch = new LatchedBoolean();
    }

    //Implements the CommandStateGenerator interface
    //All generation of subsystem demands for the CommandState are done here
    @Override
    public CommandState getCommandState() {
        CommandState state = new CommandState();
        state.setDriveDemand(generateDriveDemand());
        return state;
    }

    //This is an example of a subsystem demand generator method
    //Anything specific to this subsystem, including operator controls, is handled here
    private DriveDemand generateDriveDemand() {
        double leftDrive = -Math.abs(leftJoystick_.getY()) <= 0.1 ? -leftJoystick_.getY() : 0;
        double rightDrive = -Math.abs(rightJoystick_.getY()) <= 0.1 ? -rightJoystick_.getY() : 0;
        DriveSignal demand = new DriveSignal(leftDrive, rightDrive);
        driveLowGear = driveShiftLatch.update(rightJoystick_.getRawButton(Constants.DRIVE_TOGGLE_SHIFT_BUTTON)) ? !driveLowGear : driveLowGear;
        return new DriveDemand(demand, DriveDemand.DemandType.OpenLoop, driveLowGear);
    }

}