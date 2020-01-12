package frc2020.robot;

import frc2020.util.DriveSignal;
import frc2020.subsystems.Limelight;
import frc2020.auto.AutoChooser;
import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeRunner;
import frc2020.auto.modes.TestMode;
import frc2020.loops.*;
import frc2020.states.TeleopCSGenerator;
import frc2020.subsystems.*;
import frc2020.robot.Constants;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Looper enabledIterator;
    private Looper disabledIterator;
    private PowerDistributionPanel PDP;
    private SendableChooser<AutoChooser.AutoModeChoices> autoChooser_;
    private AutoModeRunner autoRunner_;
    private SubsystemManager manager;
    private Drive drive_;

    private Compressor compressor_;
    private AutoMode currentAutoMode_;

    private TeleopCSGenerator teleopCSGenerator_;



    private Limelight limelight_ = new Limelight();

    /**
    * Default constructor, initializes the enabledIterator, disabledIterator,
    * SubsystemManager, Drive instance, compressor, PDP, TeleopCSGenerator, and
    * the AutoChooser
    */
    public Robot() {

        enabledIterator = new Looper();
        disabledIterator = new Looper();
        autoRunner_ = null;

        manager = new SubsystemManager(
                Arrays.asList(
                  Drive.getInstance()
                )
        );


        drive_ = Drive.getInstance();
        compressor_ = new Compressor();
        PDP = new PowerDistributionPanel();
        //CSGenerators are defined here, one for teleop, one for auto (TBI)
        teleopCSGenerator_ = new TeleopCSGenerator(Constants.LEFT_DRIVER_JOYSTICK_PORT, Constants.RIGHT_DRIVER_JOYSTICK_PORT);
        autoChooser_ = AutoChooser.getAutoChooser();

        // Pre-Generate Trajectories
        TestMode.generateTrajectories();
    }

    /**
     * Logs the robot init, registers subsystem loops with the SubsystemManager,
     * outputs PDP data to SmartDashboard, and logs crashes.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            manager.registerEnabledLoops(enabledIterator);
            manager.registerDisabledLoops(disabledIterator);
            
            limelight_.setLEDMode(true);

            SmartDashboard.putData("PDP", PDP);
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
            manager.outputToSmartDashboard();
        } catch (Throwable t){
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    /**
     * Logs disabled init, stops the enabledIterator and autoRunner, enables the
     * disabledIterator, stops drive_, and logs crashes
     */
    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            enabledIterator.stop();
            if (autoRunner_ != null) {
                autoRunner_.stop();
            }
            autoRunner_ = null;
            currentAutoMode_ = null;
            disabledIterator.start();
            limelight_.setLEDMode(true);
            limelight_.setDriverMode(false);

            drive_.openLoop(new DriveSignal(0,0, false));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * Stops the disabledIterator and autoRunner, sets ramp mode and shifter mode
     * on drive, starts the enabledIterator, and logs crashes
     */
    @Override
    public void autonomousInit() {
        try {
            disabledIterator.stop();
            if (autoRunner_ != null) {
                autoRunner_.stop();
                autoRunner_ = null;
            }
            drive_.zeroSensors();
            drive_.setHighGear();
            enabledIterator.start();
            autoRunner_ = new AutoModeRunner();
            autoRunner_.setAutoMode(new TestMode());
            autoRunner_.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    /**
     * Logs teleop init, stops the disabledIterator, sets the compressor to use
     * closed loop control, starts the enabledIterator, sets ramp mode and shifter
     * mode of drive, stops the autoRunner, starts a new autoRunner, and logs crashes
     */
    @Override
    public void teleopInit() {
        try {
            System.out.println("Entering teleopInit");
            CrashTracker.logTeleopInit();
            disabledIterator.stop();
            compressor_.setClosedLoopControl(true);
            enabledIterator.start();
            drive_.zeroSensors();
            drive_.openLoop(new DriveSignal(0, 0));
            drive_.setHighGear();
            limelight_.setDriverMode(true);
            if (autoRunner_ != null) {
                autoRunner_.stop();
                autoRunner_ = null;
            }
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
     * Stops the disabledIterator, starts the enabledIterator, and logs crashes
     */
    @Override
    public void testInit() {
        try {
            System.out.println("Entering testInit");
            disabledIterator.stop();
            enabledIterator.start();
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
