package frc2020.states;

import edu.wpi.first.wpilibj.Joystick;
import frc2020.robot.Constants;
import frc2020.states.CommandState.*;
import frc2020.util.*;

/**
 * This class generates all subsystem demands to be used during teleop. All the methods
 * for generating subsystem demands should be added here to keep consistency.
 */
public class TeleopCSGenerator implements CommandStateGenerator {
    private Joystick leftJoystick_;
    private Joystick rightJoystick_;
    private LatchedBoolean driveShiftLatch;
    private boolean autoSteer = false;
    private boolean driveLowGear = false;
    
    /**
     * All ports and constants should be applied in here.
     * Anything specific to this generator should be constructed here.
     */
    public TeleopCSGenerator(int lJoyPort, int rJoyPort) {
        leftJoystick_ = new Joystick(lJoyPort);
        rightJoystick_ = new Joystick(rJoyPort);
        driveShiftLatch = new LatchedBoolean();
    }

    /**
     * Implements the CommandStateGenerator interface
     * All generation of subsystem demands for the CommandState are done here
     */
    @Override
    public CommandState getCommandState() {
        CommandState state = new CommandState();
        state.setDriveDemand(generateDriveDemand());
        return state;
    }

    /**
     * This is an example of a subsystem demand generator method
     * Anything specific to this subsystem, including operator controls, is handled here
     */
    private DriveDemand generateDriveDemand() {
        final double DEADBAND = 0.01;
        double leftDrive = Math.abs(leftJoystick_.getY()) <= DEADBAND ? 0 : -leftJoystick_.getY();
        double rightDrive = Math.abs(rightJoystick_.getY()) <= DEADBAND ? 0 : -rightJoystick_.getY();
        DriveSignal signal = new DriveSignal(leftDrive, rightDrive, true);
        driveLowGear = driveShiftLatch.update(rightJoystick_.getRawButton(Constants.DRIVE_TOGGLE_SHIFT_BUTTON)) ? !driveLowGear : driveLowGear;
        autoSteer = leftJoystick_.getRawButton(Constants.AUTO_STEER_BUTTON);
        if (autoSteer) {
            return DriveDemand.autoSteer(signal);
        }
        return DriveDemand.fromSignal(signal, driveLowGear);
    }

}
