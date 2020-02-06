package frc2020.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc2020.util.DriveSignal;
import frc2020.util.Logger;
import frc2020.util.LoggerNotStartedException;
import frc2020.subsystems.Limelight;
import frc2020.subsystems.Limelight.LedMode;
import frc2020.auto.AutoChooser;
import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeRunner;
import frc2020.auto.modes.Basic13Ball;
import frc2020.auto.modes.CenterToTrench8;
import frc2020.auto.modes.RightToTrench8;
import frc2020.auto.modes.TestMode;
import frc2020.loops.*;
import frc2020.states.TeleopCSGenerator;
import frc2020.subsystems.*;
import frc2020.robot.Constants;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private double lastFlushTime_;

    private Looper enabledIterator_;
    private Looper disabledIterator_;
    //private PowerDistributionPanel PDP;
    private SendableChooser<AutoChooser.AutoModeChoices> autoChooser_;
    private AutoModeRunner autoRunner_;
    private SubsystemManager manager_;
    private Drive drive_;

   // private Compressor compressor_;
    private AutoMode currentAutoMode_;

    private TeleopCSGenerator teleopCSGenerator_;

    private Limelight limelight_;

    private static Logger logger_ = Logger.getInstance();

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

        var limelight_config = new Limelight.LimelightConfig();
        limelight_config.height = 1.12;
        limelight_config.horizontalPlaneToLens = Rotation2d.fromDegrees(15.0);
        limelight_config.tableName = "limelight";
        limelight_config.name = "Test";
        limelight_ = new Limelight(limelight_config);
        limelight_.setLed(LedMode.PIPELINE);

        manager_ = new SubsystemManager(
                Arrays.asList(
                  Drive.getInstance(),
                  limelight_
                )
        );


        drive_ = Drive.getInstance();
       // compressor_ = new Compressor();
        //PDP = new PowerDistributionPanel();
        //CSGenerators are defined here, one for teleop, one for auto (TBI)
        teleopCSGenerator_ = new TeleopCSGenerator(Constants.LEFT_DRIVER_JOYSTICK_PORT, Constants.RIGHT_DRIVER_JOYSTICK_PORT);
        autoChooser_ = AutoChooser.getAutoChooser();

        

        // Pre-Generate Trajectories
        TestMode.generateTrajectories();
        Basic13Ball.generateTrajectories();
        CenterToTrench8.generateTrajectories();
        RightToTrench8.generateTrajectories();
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
            double now = Timer.getFPGATimestamp();
            if (now - lastFlushTime_ > Constants.LOGGER_FLUSH_TIME) {
                logger_.flush();
                lastFlushTime_ = now;
            }
            manager_.outputToSmartDashboard();
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
            drive_.openLoop(new DriveSignal(0,0, false));
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
            enabledIterator_.start();
            autoRunner_ = new AutoModeRunner();
            autoRunner_.setAutoMode(new RightToTrench8());
            autoRunner_.start();
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
            //compressor_.setClosedLoopControl(true);
            enabledIterator_.start();
            drive_.zeroSensors();
            drive_.openLoop(new DriveSignal(0, 0));
            drive_.setHighGear();
            if (autoRunner_ != null) {
                autoRunner_.stop();
                autoRunner_ = null;
            }
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
            teleopCSGenerator_.getCommandState().updateSubsystems(drive_);
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
            manager_.runPassiveTests();
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

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
}
