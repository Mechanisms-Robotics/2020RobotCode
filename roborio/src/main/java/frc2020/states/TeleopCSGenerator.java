package frc2020.states;

import java.lang.invoke.LambdaConversionException;

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
    private LatchedBoolean deployIntakeLatch;
    private boolean deployIntake;
    private boolean intakeIntake;
    private boolean outtakeIntake;
    private LatchedBoolean spinFlywheelLatch;
    private boolean spinFlywheel;
    private LatchedBoolean deployControlPanelLatch;
    private boolean deployControlPanel;
    private boolean counterClockwiseControlPanel;
    private boolean clockwiseControlPanel;


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
        deployIntakeLatch = new LatchedBoolean();
        spinFlywheelLatch = new LatchedBoolean();
        deployControlPanelLatch = new LatchedBoolean();
    }

    /**
     * Implements the CommandStateGenerator interface
     * All generation of subsystem demands for the CommandState are done here
     */
    @Override
    public CommandState getCommandState() {
        // Whether to track a power cell
        autoSteerBall = leftJoystick_.getRawButton(Constants.AUTO_STEER_BUTTON);
        // Whether to auto target to station
        autoSteerStation = leftJoystick_.getRawButton(Constants.AUTO_ALIGN_BUTTON);

        // Feeder
        intakeFeeder = rightSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_INTAKE_HAT;
        outtakeFeeder = rightSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_OUTTAKE_HAT;

        // Whether to use manual control or not
        manualControl = manualControlLatch.update(rightSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_1) && 
            rightSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_2)) != manualControl;

        // Intake
        deployIntake = deployIntakeLatch.update(rightJoystick_.getRawButton(Constants.INTAKE_DEPLOY_TOGGLE)) != deployIntake;
        intakeIntake = rightJoystick_.getTrigger();
        outtakeIntake = rightJoystick_.getRawButton(Constants.INTAKE_OUTTAKE_BUTTON);
        // This is so that if they press intake/outake and it is not deployed it will deploy
        deployIntake = (deployIntake) || (intakeIntake || outtakeIntake);

        //Flywheel
        spinFlywheel = spinFlywheelLatch.update(rightSecondJoystick_.getRawButton(Constants.FLYWHEEL_SPIN_TOGGLE)) != spinFlywheel;

        //Control Panel
        deployControlPanel = deployControlPanelLatch.update(leftSecondJoystick_.getTrigger()) != deployControlPanel;
        counterClockwiseControlPanel = leftSecondJoystick_.getPOV() != Constants.MANUAL_CONTROL_PANEL_COUNTERCLOCKWISE_HAT;
        clockwiseControlPanel = leftSecondJoystick_.getPOV() != Constants.MANUAL_CONTROL_PANEL_CLOCKWISE_HAT;

        // The command state for the robot
        CommandState state = new CommandState();
        state.setManualControl(manualControl);
        state.setLimelightDemand(generateLimelightDemand());
        state.setDriveDemand(generateDriveDemand());
        state.setFeederDemand(generateFeederDemand());
        state.setIntakeDemand(generateIntakeDemand());
        state.setFlywheelDemand(generateFlywheelDemand());
        state.setControlPanelDemand(generateControlPanelDemand());
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
            logger_.logInfo("Intake and outtake feeder hats pressed at same time", logName);
        } else if (intakeFeeder) {
            demand.intake = true;
        } else if (outtakeFeeder) {
            demand.outtake = true;
        }
        return demand;
    }

    private IntakeDemand generateIntakeDemand() {
        IntakeDemand demand = new IntakeDemand();
        if (intakeIntake && outtakeIntake) {
            logger_.logInfo("Intake and outtake intake buttons pressed at same time", logName);
        } else if (intakeIntake) {
            demand.intake = true;
        } else if (outtakeIntake) {
            demand.outtake = true;
        }

        demand.deploy = deployIntake;
        return demand;
    }

    private FlywheelDemand generateFlywheelDemand() {
        FlywheelDemand demand = new FlywheelDemand();

        demand.spin = spinFlywheel;

        return demand;
    }

    private ControlPanelDemand generateControlPanelDemand() {
        ControlPanelDemand demand = new ControlPanelDemand();

        if (clockwiseControlPanel && counterClockwiseControlPanel) {
            logger_.logInfo("Counterclockwise and clockwise control panel buttons pressed at same time", logName);
        } else if (clockwiseControlPanel) {
        demand.clockwise = true;
        } else if (counterClockwiseControlPanel) {
        demand.counterclockwise = counterClockwiseControlPanel;
        }

        demand.deploy = deployControlPanel;
        return demand;
    }

    public synchronized void disableManualControl() {
        manualControl = false;
    }
}
