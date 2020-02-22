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

    // ONLY PERSISTENT VALUES SHOULD BE STORED HERE
    private final double JOYSTICK_DEADBAND = 0.01;
    private LatchedBoolean driveShiftLatch;
    private boolean autoSteerBall = false;
    private boolean autoSteerStation = false;
    private boolean driveLowGear = false;

    private LatchedBoolean manualControlLatch;
    private boolean manualControl = false;
    private LatchedBoolean deployIntakeLatch;
    private boolean deployIntake = false;
    private LatchedBoolean spinFlywheelLatch;
    private boolean spinFlywheel = false;

    private LatchedBoolean deployClimberLatch;
    private boolean deployClimber = false;
    private LatchedBoolean lockClimberLatch;
    private boolean lockClimber = false;

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
        deployClimberLatch = new LatchedBoolean();
        lockClimberLatch = new LatchedBoolean();
        spinFlywheelLatch = new LatchedBoolean();
        deployClimberLatch = new LatchedBoolean();
        lockClimberLatch = new LatchedBoolean();
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

        // Whether to use manual control or not
        manualControl = manualControlLatch.update(rightSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_1) &&
                rightSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_2)) != manualControl;

        // The command state for the robot
        CommandState state = new CommandState();
        state.setManualControl(manualControl);
        state.setLimelightDemand(generateLimelightDemand());
        state.setDriveDemand(generateDriveDemand());
        state.setFeederDemand(generateFeederDemand());
        state.setIntakeDemand(generateIntakeDemand());
        state.setFlywheelDemand(generateFlywheelDemand());
        state.setTurretDemand(generateTurretDemand());
        state.setClimberDemand(generateClimberDemand());
        return state;
    }

    /**
     * This is an example of a subsystem demand generator method
     * Anything specific to this subsystem, including operator controls, is handled here
     */
    private DriveDemand generateDriveDemand() {
        //Drive
        driveLowGear = driveShiftLatch.update(rightJoystick_.getRawButton(Constants.DRIVE_TOGGLE_SHIFT_BUTTON)) != driveLowGear;
        double leftDrive = Math.abs(leftJoystick_.getY()) <= JOYSTICK_DEADBAND ? 0 : -leftJoystick_.getY();
        double rightDrive = Math.abs(rightJoystick_.getY()) <= JOYSTICK_DEADBAND ? 0 : -rightJoystick_.getY();

        DriveSignal signal = new DriveSignal(leftDrive, rightDrive, true);
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
        boolean intakeFeeder = rightSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_INTAKE_HAT;
        boolean outtakeFeeder = rightSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_OUTTAKE_HAT;

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
        deployIntake = deployIntakeLatch.update(rightJoystick_.getTrigger()) != deployIntake;
        boolean intakeIntake = rightJoystick_.getRawButton(Constants.INTAKE_INTAKE_BUTTON);
        boolean outtakeIntake = rightJoystick_.getRawButton(Constants.INTAKE_OUTTAKE_BUTTON);

        // This is so that if they press intake/outake and it is not deployed it will deploy
        deployIntake = (deployIntake) || (intakeIntake || outtakeIntake);

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
        demand.spin = spinFlywheelLatch.update(rightSecondJoystick_.getRawButton(Constants.FLYWHEEL_SPIN_TOGGLE)) != spinFlywheel;
        return demand;
    }

    private TurretDemand generateTurretDemand() {
        TurretDemand demand = new TurretDemand();
        double turretDeadband = 0.12;
        double maxSpeed = 0.25;
        demand.useOpenLoop = true;
        demand.speed = Util.limit(Math.abs(leftSecondJoystick_.getY()) <= turretDeadband ? 0 : leftSecondJoystick_.getY(), -maxSpeed, maxSpeed);
        return demand;
    }

    private ClimberDemand generateClimberDemand() {
        boolean deployButtonsPressed = rightSecondJoystick_.getRawButton(Constants.DEPLOY_CLIMBER_TOGGLE_1) &&
                rightSecondJoystick_.getRawButton(Constants.DEPLOY_CLIMBER_TOGGLE_2);

        ClimberDemand demand = new ClimberDemand();
        demand.deploy = deployClimberLatch.update(deployButtonsPressed) != deployClimber;;
        demand.lock = lockClimberLatch.update(rightSecondJoystick_.getRawButton(Constants.LOCK_CLIMBER_TOGGLE)) != lockClimber;;
        demand.winchSpeed = Math.abs(rightSecondJoystick_.getY()) <= JOYSTICK_DEADBAND ? 0 : -rightSecondJoystick_.getY();

        return demand;
    }

    public synchronized void disableManualControl() {
        manualControl = false;
    }
}
