package frc2020.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.auto.modes.Basic3Ball;
import frc2020.util.*;
import frc2020.subsystems.Limelight;
import frc2020.subsystems.Limelight.LedMode;
import frc2020.auto.AutoChooser;
import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeRunner;
import frc2020.auto.modes.CenterToTrench8;
import frc2020.auto.modes.IntakeTestingAuto;
import frc2020.auto.modes.RightToTrench8;
import frc2020.auto.modes.AutoAward;
import frc2020.auto.modes.BouncePath;
import frc2020.auto.modes.BarrelRacing;
import frc2020.auto.modes.Slalom;
import frc2020.loops.*;
import frc2020.states.TeleopCSGenerator;
import frc2020.subsystems.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;
import java.util.UUID;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    private UUID runUUID_;

    private Looper enabledIterator_;
    private Looper disabledIterator_;
    //private PowerDistributionPanel PDP;
    private SendableChooser<AutoChooser.AutoModeChoices> autoChooser_;
    private AutoModeRunner autoRunner_;
    private SubsystemManager manager_;
    private Drive drive_;
    private Intake intake_;
    private Feeder feeder_;
    private Flywheel flywheel_;
    private Climber climber_;
    private Turret turret_;
    private Hood hood_;
    private Shooter shooter_;
    private ControlPanel controlPanel_;
    private FloodGate floodGate_;

    private Compressor compressor_;
    private DriverStation ds_;
    private AutoMode currentAutoMode_;

    private TeleopCSGenerator teleopCSGenerator_;

    private Limelight limelight_turret_;
    private Limelight limelight_low_;

    private static Logger logger_ = Logger.getInstance();

    private PeriodicEventManager periodicEventManager_ = new PeriodicEventManager();

    /**
    * Default constructor, initializes the enabledIterator_, disabledIterator_,
    * SubsystemManager, Drive instance, compressor, PDP, TeleopCSGenerator, and
    * the AutoChooser
    */
    public Robot() {

        runUUID_ = UUID.randomUUID();
        logger_.start(runUUID_,
         "RobotLog", Logger.Level.Debug);
        enabledIterator_ = new Looper();
        disabledIterator_ = new Looper();
        autoRunner_ = null;

        var limelight_turret_config = new Limelight.LimelightConfig();
        var limelight_low_config = new Limelight.LimelightConfig();

        limelight_turret_config.height = 0.592;
        limelight_turret_config.horizontalPlaneToLens = Rotation2d.fromDegrees(20.0);
        limelight_turret_config.tableName = "limelight-turret";
        limelight_turret_config.name = "Limelight Turret";
        limelight_turret_config.azimuthOnly = false;

        limelight_low_config.height = 0.61;
        limelight_low_config.horizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
        limelight_low_config.tableName = "limelight-low";
        limelight_low_config.name = "Limelight Low";
        limelight_low_config.azimuthOnly = true;

        limelight_turret_ = new Limelight(limelight_turret_config);
        limelight_low_ = new Limelight(limelight_low_config);
        limelight_turret_.setLed(LedMode.PIPELINE);
        limelight_low_.setLed(LedMode.PIPELINE);

        manager_ = new SubsystemManager(
                Arrays.asList(
                  Drive.getInstance(),
                  limelight_turret_,
                  limelight_low_,
                  // TODO: Put subystems here once tuned
                  Feeder.getInstance(),
                  Intake.getInstance(),
                  Climber.getInstance(),
                  Turret.getInstance(),
                  Hood.getInstance(),
                  Flywheel.getInstance(),
                  Shooter.getInstance(),
                  ControlPanel.getInstance(),
                  FloodGate.getInstance()
                )
        );

        drive_ = Drive.getInstance();
        intake_ = Intake.getInstance();
        feeder_ = Feeder.getInstance();
        turret_ = Turret.getInstance();
        flywheel_ = Flywheel.getInstance();
        climber_ = Climber.getInstance();
        hood_ = Hood.getInstance();
        controlPanel_ = ControlPanel.getInstance();
        floodGate_ = FloodGate.getInstance();
        shooter_ = Shooter.getInstance();
        shooter_.setLimelight(limelight_turret_);

        compressor_ = new Compressor();
        ds_ = DriverStation.getInstance();
        //PDP = new PowerDistributionPanel();
        //CSGenerators are defined here, one for teleop, one for auto (TBI)
        teleopCSGenerator_ = new TeleopCSGenerator(Constants.LEFT_DRIVER_JOYSTICK_PORT, Constants.RIGHT_DRIVER_JOYSTICK_PORT,
            Constants.LEFT_SECONDARY_DRIVER_JOYSTICK_PORT, Constants.RIGHT_SECONDARY_DRIVER_JOYSTICK_PORT);
        autoChooser_ = AutoChooser.getAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser_);

        // Pre-Generate Trajectories
        //Basic13Ball.generateTrajectories();
        Basic3Ball.generateTrajectories();
        CenterToTrench8.generateTrajectories();
        RightToTrench8.generateTrajectories();
        AutoAward.generateTrajectories();
        BouncePath.generateTrajectories();
        Slalom.generateTrajectories();
        IntakeTestingAuto.generateTrajectories();
        BarrelRacing.generateTrajectories();

        PeriodicEvent flushLog_ = new PeriodicEvent(){
            @Override
            public void run() {
                logger_.flush();
            }
        
            @Override
            public boolean condition() {
                return true;
            }
        };

        PeriodicEvent runPassiveTests_ = new PeriodicEvent() {
            @Override
            public void run() {
                manager_.runPassiveTests();
            }
        
            @Override
            public boolean condition() {
                return m_ds.isDisabled();
            }
        };

        periodicEventManager_.addEvent(flushLog_, Constants.LOGGER_FLUSH_TIME);
        periodicEventManager_.addEvent(runPassiveTests_, Constants.PASSIVE_TEST_TIME);
    }

    /**
     * Logs the robot init, registers subsystem loops with the SubsystemManager,
     * outputs PDP data to SmartDashboard, and logs crashes.
     */
    @Override
    public void robotInit() {
        try {
            logger_.logRobotInit();
            CrashTracker.logRobotInit();

            manager_.registerEnabledLoops(enabledIterator_);
            manager_.registerDisabledLoops(disabledIterator_);
            limelight_turret_.setPipeline(0);
            limelight_low_.setPipeline(2); // TODO: Remove harcoded pipline change (and spelling errors)
            
            //SmartDashboard.putData("PDP", PDP);
        } catch(LoggerNotStartedException e) {
            logger_.setFileLogging(false);
            DriverStation.reportError(
                    "Unable to start logging: Disabling File Logging(will still print out messages to console)",
                    false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Tells the SubsystemManager to get all subsystems to output their telemetry
     * to the SmartDashboard and logs crashes.
     */
    @Override
    public void robotPeriodic() {
        try {

            periodicEventManager_.run();
            manager_.outputToSmartDashboard();
            SmartDashboard.putBoolean("IsCompBot: ", Constants.IS_COMP_BOT);
            SmartDashboard.putNumber("MATCH TIME", ds_.getMatchTime());
//            Pose2d target = targetTracker_.getRobotToVisionTarget();
//            if (target != null) {
//                SmartDashboard.putNumber("Distance", target.getTranslation().getX());
//            } else {
//                SmartDashboard.putNumber("Distance", 0.0);
//            }
        } catch (Throwable t){
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Logs disabled init, stops the enabledIterator_ and autoRunner, enables the
     * disabledIterator_, stops drive_, and logs crashes
     */
    @Override
    public void disabledInit() {
        try {
            logger_.logRobotDisabled();
            CrashTracker.logDisabledInit();
            enabledIterator_.stop();
            if (autoRunner_ != null) {
                autoRunner_.stop();
            }
            autoRunner_ = null;
            currentAutoMode_ = null;
            disabledIterator_.start();
            drive_.openLoop(new DriveSignal(0, 0, false));
            limelight_turret_.setLed(LedMode.OFF);
            teleopCSGenerator_.resetManualControl();
            climber_.lockWinch();
        } catch(LoggerNotStartedException e) {
            logger_.setFileLogging(false);
            DriverStation.reportError(
                    "Unable to start logging: Disabling File Logging(will still print out messages to console)",
                    false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * Stops the disabledIterator_ and autoRunner, sets ramp mode and shifter mode
     * on drive, starts the enabledIterator_, and logs crashes
     */
    @Override
    public void autonomousInit() {
        try {
            logger_.logRobotAutoInit();
            disabledIterator_.stop();
            if (autoRunner_ != null) {
                autoRunner_.stop();
                autoRunner_ = null;
            }
            drive_.zeroSensors();
            drive_.setHighGear();
            teleopCSGenerator_.resetManualControl();
            enabledIterator_.start();
            autoRunner_ = new AutoModeRunner();
            autoRunner_.setAutoMode(AutoChooser.getAuto(autoChooser_.getSelected()));
            autoRunner_.start();
            climber_.resetHasDeployed();
        } catch(LoggerNotStartedException e) {
            logger_.setFileLogging(false);
            DriverStation.reportError(
                    "Unable to start logging: Disabling File Logging(will still print out messages to console)",
                    false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    /**
     * Logs teleop init, stops the disabledIterator_, sets the compressor to use
     * closed loop control, starts the enabledIterator_, sets ramp mode and shifter
     * mode of drive, stops the autoRunner, starts a new autoRunner, and logs crashes
     */
    @Override
    public void teleopInit() {
        try {
            logger_.logRobotTeleopInit();
            CrashTracker.logTeleopInit();
            disabledIterator_.stop();
            compressor_.setClosedLoopControl(true);
            enabledIterator_.start();
            drive_.zeroSensors();
            drive_.openLoop(new DriveSignal(0, 0));
            drive_.setHighGear();
            climber_.resetHasDeployed();
            climber_.unlockWinch();
            if (autoRunner_ != null) {
                autoRunner_.stop();
                autoRunner_ = null;
            }
            teleopCSGenerator_.resetManualControl();
            teleopCSGenerator_.resetPresetPositions();
            shooter_.handleReenable();
        } catch(LoggerNotStartedException e) {
            logger_.setFileLogging(false);
            DriverStation.reportError(
                    "Unable to start logging: Disabling File Logging(will still print out messages to console)",
                    false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
    * Tells the TeleopCSGenerator to update subsystems and logs crashes
    */
    @Override
    public void teleopPeriodic() {
        try {
            //This one line of code handles all teleoperated control
            //Add subsystems to the updateSubsystems method to expand as needed
            teleopCSGenerator_.getCommandState().updateSubsystems(
                    drive_,
                    limelight_low_,
                    feeder_,
                    turret_,
                    intake_,
                    flywheel_,
                    climber_,
                    hood_,
                    shooter_,
                    controlPanel_,
                    floodGate_);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Stops the disabledIterator_, starts the enabledIterator_, and logs crashes
     */
    @Override
    public void testInit() {
        try {
            logger_.logRobotTestInit();
            disabledIterator_.stop();
            enabledIterator_.start();
            teleopCSGenerator_.resetManualControl();
  //          manager_.runActiveTests();
        } catch (Throwable t){
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Logs crashes
     */
    @Override
    public void testPeriodic() {
        try {
            //controlPanel_.setOpenLoop(1);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
}
