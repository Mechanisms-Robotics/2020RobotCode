package frc2020.states;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.robot.Constants;
import frc2020.states.CommandState.*;
import frc2020.subsystems.Drive;
import frc2020.subsystems.Limelight;
import frc2020.subsystems.Shooter;
import frc2020.subsystems.Shooter.ShooterState;
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

    private Drive drive_;

    // ONLY PERSISTENT VALUES SHOULD BE STORED HERE
    private final double JOYSTICK_DEADBAND = 0.01;
    private LatchedBoolean driveShiftLatch;
    private boolean autoSteerBall = false;
    private boolean autoSteerStation = false;
    private boolean driveLowGear = false;
    private LatchedBoolean autoBackupLatch;
    private boolean autoBackup = false;
    private boolean autoBackupLatchBoolean = false;

    private LatchedBoolean manualControlLatch;
    private boolean manualControl = false;
    private LatchedBoolean deployIntakeLatch;
    private boolean deployIntake = false;

    private LatchedBoolean spinFlywheelLatch;
    private boolean spinFlywheel = false;

    private LatchedBoolean deployControlPanelLatch;
    private boolean deployControlPanel = false;
    private LatchedBoolean controlPanelRotationLatch;
    private LatchedBoolean controlPanelPositionLatch;

    private LatchedBoolean deployClimberLatch;
    private boolean deployClimber = false;
    private LatchedBoolean lockClimberLatch;
    private boolean lockClimber = false;
    private LatchedBoolean deployHoodLatch;
    private boolean deployHood = false;
    private LatchedBoolean climberSplitLatch;
    private boolean climberSplit = false;

    private LatchedBoolean getStowAimingLatch;
    private LatchedBoolean getShooterLatch;
    private LatchedBoolean getTrenchLatch;
    private boolean getTrench = false;

    private boolean isFeederDemand = false;

    private Shooter shooter_ = Shooter.getInstance();

    private CheesyDriveHelper cheesyHelper_;

    private Logger logger_ = Logger.getInstance();
    private String logName = "TeleopCS";


    private enum DriveMode {
        Tank,
        Arcade,
        Cheesy
    }
    private SendableChooser<DriveMode> driveChooser;

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
        autoBackupLatch = new LatchedBoolean();
        manualControlLatch = new LatchedBoolean();
        deployIntakeLatch = new LatchedBoolean();
        deployClimberLatch = new LatchedBoolean();
        lockClimberLatch = new LatchedBoolean();
        spinFlywheelLatch = new LatchedBoolean();
        deployControlPanelLatch = new LatchedBoolean();
        controlPanelRotationLatch = new LatchedBoolean();
        controlPanelPositionLatch = new LatchedBoolean();

        drive_ = Drive.getInstance();
        cheesyHelper_ = new CheesyDriveHelper();
        deployClimberLatch = new LatchedBoolean();
        lockClimberLatch = new LatchedBoolean();
        deployHoodLatch = new LatchedBoolean();
        getStowAimingLatch = new LatchedBoolean();
        getShooterLatch = new LatchedBoolean();
        getTrenchLatch = new LatchedBoolean();

        drive_ = Drive.getInstance();

        climberSplitLatch = new LatchedBoolean();
        driveChooser = new SendableChooser<>();
        driveChooser.setDefaultOption(DriveMode.Tank.toString(), DriveMode.Tank);
        driveChooser.addOption(DriveMode.Arcade.toString(), DriveMode.Arcade);
        driveChooser.addOption(DriveMode.Cheesy.toString(), DriveMode.Cheesy);
        SmartDashboard.putData("Drive Chooser", driveChooser);
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
        // Whether to run the power port backup sequence
        boolean autoBackupSharedBoolean  = autoBackupLatch.update(rightJoystick_.getPOV() == Constants.AUTO_BACKUP_POV_HAT);
        autoBackup = autoBackupSharedBoolean != autoBackup;
        autoBackupLatchBoolean = autoBackupSharedBoolean;

        // Whether to use manual control or not
        manualControl = manualControlLatch.update(leftSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_1) &&
                leftSecondJoystick_.getRawButton(Constants.MANUAL_CONTROL_BUTTON_2)) != manualControl;

        // The command state for the robot
        CommandState state = new CommandState();
        state.setManualControl(manualControl);
        state.setLimelightDemand(generateLimelightDemand());
        state.setDriveDemand(generateDriveDemand());
        state.setShooterDemand(generateShooterDemand());
        state.setClimberDemand(generateClimberDemand());
        state.setIntakeDemand(generateIntakeDemand());
        state.setFeederDemand(generateFeederDemand());
        if (manualControl) {
            state.setFlywheelDemand(generateFlywheelDemand());
            state.setTurretDemand(generateTurretDemand());
            state.setHoodDemand(generateHoodDemand());
        } else {
            resetManualControl();
        }

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
        final double BACKUP_DISTANCE = 0.54;
        //Drive
        driveLowGear = driveShiftLatch.update(rightJoystick_.getRawButton(Constants.DRIVE_TOGGLE_SHIFT_BUTTON)) != driveLowGear;

        final double DEADBAND = 0.01;

        double leftDrive = 0.0;
        double rightDrive = 0.0;

        var driveMode = driveChooser.getSelected();
        var povDir = leftJoystick_.getPOV();
        var quickTurn = povDir == 0 || povDir == 45 || povDir == 315;

        if (driveMode == DriveMode.Tank) {
            leftDrive = Math.abs(leftJoystick_.getY()) <= DEADBAND ? 0 : -leftJoystick_.getY();
            rightDrive = Math.abs(rightJoystick_.getY()) <= DEADBAND ? 0 : -rightJoystick_.getY();
            int lSign = 1;
            int rSign = 1;
            if (leftDrive < 0) {
                lSign = -1;
            }
            if (rightDrive < 0) {
                rSign = -1;
            }
            final double JOYSTICK_EXPONENT = 1.7;
            leftDrive = Math.pow(Math.abs(leftDrive), JOYSTICK_EXPONENT) * lSign;
            rightDrive = Math.pow(Math.abs(rightDrive), JOYSTICK_EXPONENT) * rSign;
        } else if (driveMode == DriveMode.Arcade) {
            leftDrive = Math.abs(leftJoystick_.getY()) <= DEADBAND ? 0 : -leftJoystick_.getY();
            rightDrive = Math.abs(leftJoystick_.getY()) <= DEADBAND ? 0 : -leftJoystick_.getY();
            leftDrive += Math.abs(rightJoystick_.getX()) <= DEADBAND ? 0 : rightJoystick_.getX()*0.75f;
            rightDrive -= Math.abs(rightJoystick_.getX()) <= DEADBAND ? 0 : rightJoystick_.getX()*0.75f;
        } else if (driveMode == DriveMode.Cheesy){
            double throttle = Math.abs(leftJoystick_.getY()) <= DEADBAND ? 0 : -leftJoystick_.getY();
            double wheel = Math.abs(rightJoystick_.getX()) <= DEADBAND ? 0 : rightJoystick_.getX();
            DriveSignal dSignal = cheesyHelper_.cheesyDrive(throttle, wheel, quickTurn, !driveLowGear);
            leftDrive = dSignal.getLeft();
            rightDrive = dSignal.getRight();
        } else {
            logger_.logWarning("Invalid drive mode!", logName);
        }

        DriveSignal signal = new DriveSignal(leftDrive, rightDrive, true);
        if (autoSteerBall || autoSteerStation) {
            return DriveDemand.autoSteer(signal);
        }
        if (autoBackup) {
            if (autoBackupLatchBoolean) {
                drive_.setBackupDistance(BACKUP_DISTANCE);
            }
            return DriveDemand.autoBackup();
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
        boolean intakeFeeder = leftSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_INTAKE_HAT;
        boolean outtakeFeeder = leftSecondJoystick_.getPOV() == Constants.MANUAL_FEEDER_OUTTAKE_HAT;

        FeederDemand demand = new FeederDemand();
        if (intakeFeeder && outtakeFeeder) {
            logger_.logInfo("Intake and outtake feeder hats pressed at same time", logName);
        } else if (intakeFeeder) {
            demand.intake = true;
        } else if (outtakeFeeder) {
            demand.outtake = true;
        }

        isFeederDemand = intakeFeeder || outtakeFeeder;
        return demand;
    }

    private IntakeDemand generateIntakeDemand() {
        deployIntake = deployIntakeLatch.update(rightJoystick_.getTrigger()) != deployIntake;
        boolean intakeIntake = rightJoystick_.getRawButton(Constants.INTAKE_INTAKE_BUTTON);
        boolean outtakeIntake = rightJoystick_.getRawButton(Constants.INTAKE_OUTTAKE_BUTTON);

        // This is so that if they press intake/outake and it is not deployed it will deploy
        //deployIntake = (deployIntake) || (intakeIntake || outtakeIntake);

        IntakeDemand demand = new IntakeDemand();
        // if (intakeIntake && outtakeIntake) {
        //     logger_.logInfo("Intake and outtake intake buttons pressed at same time", logName);
        // } else if (intakeIntake) {
        //     demand.intake = true;
        // } else if (outtakeIntake) {
        //     demand.outtake = true;
        // }

        demand.deploy = deployIntake;
        demand.intake = deployIntake;
        if(outtakeIntake) {
            demand.intake = false;
            demand.outtake = true;
        }
        return demand;
    }

    private FlywheelDemand generateFlywheelDemand() {
        FlywheelDemand demand = new FlywheelDemand();
        spinFlywheel = spinFlywheelLatch.update(leftSecondJoystick_.getRawButton(Constants.FLYWHEEL_SPIN_TOGGLE)) != spinFlywheel;
        demand.spin = spinFlywheel;
        return demand;
    }

    private TurretDemand generateTurretDemand() {
        final double TURRET_DEADBAND = 0.12;
        final double MAX_SPEED = 0.25;
        TurretDemand demand = new TurretDemand();
        demand.useOpenLoop = true;
        demand.speed = Math.abs(rightSecondJoystick_.getTwist()) <= TURRET_DEADBAND ? 0 : -rightSecondJoystick_.getTwist()*MAX_SPEED;
        return demand;
    }

    private ClimberDemand generateClimberDemand() {
        ClimberDemand demand = new ClimberDemand();
        boolean deployButtonsPressed = rightSecondJoystick_.getRawButton(Constants.DEPLOY_CLIMBER_TOGGLE_1) &&
        rightSecondJoystick_.getRawButton(Constants.DEPLOY_CLIMBER_TOGGLE_2);

        double leftWinchSpeed = Math.abs(leftSecondJoystick_.getY()) <= JOYSTICK_DEADBAND ? 0 : -leftSecondJoystick_.getY();
        double rightWinchSpeed = Math.abs(rightSecondJoystick_.getY()) <= JOYSTICK_DEADBAND ? 0 : -rightSecondJoystick_.getY();

        climberSplit = climberSplitLatch.update(rightSecondJoystick_.getTrigger()) != climberSplit;
        deployClimber = deployClimberLatch.update(deployButtonsPressed) != deployClimber;
        demand.deploy = deployClimber;
        lockClimber = lockClimberLatch.update(rightSecondJoystick_.getRawButton(Constants.LOCK_CLIMBER_TOGGLE)) != lockClimber;
        demand.lock = lockClimber;
        demand.leftWinchSpeed = climberSplit ? leftWinchSpeed : rightWinchSpeed;
        demand.rightWinchSpeed = rightWinchSpeed;
        return demand;
    }

    private ControlPanelDemand generateControlPanelDemand() {
        ControlPanelDemand demand = new ControlPanelDemand();

        deployControlPanel = deployControlPanelLatch.update(rightSecondJoystick_.getRawButton(Constants.DEPLOY_CONTROL_PANEL_TOGGLE)) != deployControlPanel;
        boolean controlPanelRotation = controlPanelRotationLatch.update(rightSecondJoystick_.getRawButton(Constants.CONTROL_PANEL_ROTATION_TOGGLE));
        boolean controlPanelPosition = controlPanelPositionLatch.update(rightSecondJoystick_.getRawButton(Constants.CONTROL_PANEL_POSITION_TOGGLE));

        boolean counterClockwiseControlPanel = rightSecondJoystick_.getPOV() == Constants.MANUAL_CONTROL_PANEL_COUNTERCLOCKWISE_HAT;
        boolean clockwiseControlPanel = rightSecondJoystick_.getPOV() == Constants.MANUAL_CONTROL_PANEL_CLOCKWISE_HAT;

        if (clockwiseControlPanel && counterClockwiseControlPanel) {
            logger_.logInfo("Counterclockwise and clockwise control panel buttons pressed at same time", logName);
        } else if (clockwiseControlPanel) {
            demand.clockwise = true;
        } else if (counterClockwiseControlPanel) {
            demand.counterclockwise = true;
        }

        if (!(controlPanelPosition && controlPanelRotation)) {
            if (controlPanelRotation) {
                demand.rotation = true;
            } else if (controlPanelPosition) {
                demand.position = true;
            }
        }

        demand.deploy = deployControlPanel;
        return demand;
    }
    private HoodDemand generateHoodDemand() {
        HoodDemand demand = new HoodDemand();
        final double MAX_SPEED = 0.1;

        deployHood = deployHoodLatch.update(leftSecondJoystick_.getTrigger()) != deployHood;

        demand.speed = Util.limit(Math.abs(leftSecondJoystick_.getY()) <= JOYSTICK_DEADBAND ? 0 : leftSecondJoystick_.getY(),
                                    -MAX_SPEED, MAX_SPEED);
        demand.deploy = deployHood;
        return demand;
    }

    private ShooterDemand generateShooterDemand() {

        ShooterDemand demand = new ShooterDemand();

        boolean getStowAiming = getStowAimingLatch.update(leftJoystick_.getRawButton(Constants.SHOOTER_SET_STOWED_AIMING));
        boolean getShooter = getShooterLatch.update(leftJoystick_.getTrigger());

        getTrench = getTrenchLatch.update(rightJoystick_.getPOV() == Constants.TRENCH_POV_HAT) != getTrench;

        if (manualControl) {
            demand.state = ShooterState.Manual;
        } else {
            if (shooter_.getWantedState() == ShooterState.Aiming || shooter_.getWantedState() == ShooterState.Shooting) {
                if (getStowAiming) {
                    demand.state = ShooterState.Stowed;
                } else if (getShooter) {
                    if (shooter_.getWantedState() != ShooterState.Shooting) {
                        demand.state = ShooterState.Shooting;
                    } else {
                        demand.state = ShooterState.Stowed;
                    }
                } else {
                    demand.state = shooter_.getWantedState();
                }
            } else if (shooter_.getWantedState() == ShooterState.PowerPort) {
                if (!autoBackup) {
                    demand.state = ShooterState.Stowed;
                } else {
                    demand.state = ShooterState.PowerPort;
                }
            } else if (shooter_.getWantedState() == ShooterState.Trench) {
                if (!getTrench) {
                    demand.state = ShooterState.Stowed;
                } else {
                    demand.state = ShooterState.Trench;
                }
            } else {
                if (getStowAiming) {
                    demand.state = ShooterState.Aiming;
                } else if (getShooter) {
                    demand.state = ShooterState.Shooting;
                } else if (autoBackupLatchBoolean) {
                    demand.state = ShooterState.PowerPort;
                } else if (getTrench) {
                    demand.state = ShooterState.Trench;
                }
            }
        }

        demand.overrideFeeder = isFeederDemand;

        return demand;
    }

    public synchronized void resetManualControl() {
        manualControl = false;
        deployHood = false;
        spinFlywheel = false;
    }

    public synchronized void resetPresetPositions() {
        autoBackup = false;
        getTrench = false;
    }
}
