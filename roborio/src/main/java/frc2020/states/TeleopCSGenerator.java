package frc2020.states;

import edu.wpi.first.wpilibj.Joystick;
import frc2020.robot.Constants;
import frc2020.states.CommandState.*;
import frc2020.subsystems.Limelight;
import frc2020.util.*;

/**
 * This class generates all subsystem demands to be used during teleop. All the methods
 * for generating subsystem demands should be added here to keep consistency.
 */
public class TeleopCSGenerator implements CommandStateGenerator {
    private Joystick leftJoystick_;
    private Joystick rightJoystick_;
    private Joystick leftSecondJoystick_;
    private Joystick rightSecondJoystick_;
    private LatchedBoolean driveShiftLatch;
    private boolean autoSteerBall = false;
    private boolean autoSteerStation = false;
    private boolean driveLowGear = false;
    private boolean intakeFeeder = false;
    private boolean outtakeFeeder = false;
    private LatchedBoolean manualControlLatch;
    private boolean manualControl;

    private Logger logger_ = Logger.getInstance();
    private String logName = "TeleopCS";
    
    /**
     * All ports and constants should be applied in here.
     * Anything specific to this generator should be constructed here.
     */
    public TeleopCSGenerator(int lJoyPort, int rJoyPort, int lJoySecPort, int rJoySecPort) {
        leftJoystick_ = new Joystick(lJoyPort);
        rightJoystick_ = new Joystick(rJoyPort);
        leftSecondJoystick_ = new Joystick(lJoySecPort);
        rightSecondJoystick_ = new Joystick(rJoySecPort);
        driveShiftLatch = new LatchedBoolean();
        manualControlLatch = new LatchedBoolean();
    }

    /**
     * Implements the CommandStateGenerator interface
     * All generation of subsystem demands for the CommandState are done here
     */
    @Override
    public CommandState getCommandState() {
        autoSteerBall = leftJoystick_.getRawButton(Constants.AUTO_STEER_BUTTON);
        autoSteerStation = leftJoystick_.getRawButton(Constants.AUTO_ALIGN_BUTTON);
        intakeFeeder = rightSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_INTAKE_HAT;
        outtakeFeeder = rightSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_OUTTAKE_HAT;
        manualControl = manualControlLatch.update(rightSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_1) && 
            rightSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_2)) != manualControl;
        CommandState state = new CommandState();
        state.setManualControl(manualControl);
        state.setLimelightDemand(generateLimelightDemand());
        state.setDriveDemand(generateDriveDemand());
        state.setFeederDemand(generateFeederDemand());
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
        driveLowGear = driveShiftLatch.update(rightJoystick_.getRawButton(Constants.DRIVE_TOGGLE_SHIFT_BUTTON)) != driveLowGear;
        if (autoSteerBall || autoSteerStation) {
            return DriveDemand.autoSteer(signal);
        }
        return DriveDemand.fromSignal(signal, driveLowGear);
    }

    private LimelightDemand generateLimelightDemand() {
        LimelightDemand demand = new LimelightDemand();
        if (autoSteerBall) {
            demand.ledMode = Limelight.LedMode.PIPELINE;
            demand.pipeline = Constants.POWER_CELL_PIPELINE;
        } else if (autoSteerStation) {
            demand.ledMode = Limelight.LedMode.PIPELINE;
            demand.pipeline = Constants.LOADING_STATION_PIPELINE;
        } else {
            demand.ledMode = Limelight.LedMode.OFF;
            demand.pipeline = Constants.DRIVER_MODE_PIPELINE;
        }
        return demand;
    }

    private FeederDemand generateFeederDemand() {
        FeederDemand demand = new FeederDemand();
        if (intakeFeeder && outtakeFeeder) {
            logger_.logInfo("Intake and outtake hat pressed at same time", logName);
        } else if (intakeFeeder) {
            demand.intake = true;
        } else if (outtakeFeeder) {
            demand.outtake = true;
        }
        return demand;
    }
}
