package frc2020.robot;

import frc2020.util.DriveSignal;
import frc2020.util.geometry.Pose2d;
import frc2020.auto.AutoChooser;
import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeRunner;
import frc2020.loops.*;
import frc2020.states.AutoCSGenerator;
import frc2020.states.TeleopCSGenerator;
import frc2020.subsystems.*;

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
    private Jetson jetson_;
    private Compressor compressor_;
    private AutoMode currentAutoMode_;

    private TeleopCSGenerator teleopCSGenerator_;
    private AutoCSGenerator autoCSGenerator_;

    public Robot() {

        enabledIterator = new Looper();
        disabledIterator = new Looper();
        autoRunner_ = null;

        manager = new SubsystemManager(
                Arrays.asList(
                  Drive.getInstance(),
                  Jetson.getInstance(),
                  RobotStateEstimator.getInstance()
                )
        );
        

        drive_ = Drive.getInstance();
        jetson_ = Jetson.getInstance();
        compressor_ = new Compressor();
        PDP = new PowerDistributionPanel();
        //CSGenerators are defined here, one for teleop, one for auto (TBI)
        teleopCSGenerator_ = new TeleopCSGenerator(Constants.LEFT_DRIVER_JOYSTICK_PORT, Constants.RIGHT_DRIVER_JOYSTICK_PORT);
        autoCSGenerator_ = new AutoCSGenerator(jetson_);
        autoChooser_ = AutoChooser.getAutoChooser();
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            manager.registerEnabledLoops(enabledIterator);
            manager.registerDisabledLoops(disabledIterator);

            SmartDashboard.putData("PDP", PDP);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * The periodic function that runs for the entire runtime of the robot
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
     * This function preforms all tasks that are needed to disable the robot
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
            drive_.openLoop(new DriveSignal(0,0, false));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Any periotic tasks that need to run while the robot is disabled
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function configures the robot for atonomous and starts the atuo thread
     */
    @Override
    public void autonomousInit() {
        try {
            disabledIterator.stop();
            if (autoRunner_ != null) {
                autoRunner_.stop();
                autoRunner_ = null;
            }
            drive_.setRampMode(Drive.RampMode.None);
            drive_.setHighGear();
            enabledIterator.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        autoCSGenerator_.getCommandState().updateSubsystems(drive_);
    }

    /**
     * Configures the robot for teleop
     */
    @Override
    public void teleopInit() {
        try {
            System.out.println("Entering teleopInit");
            CrashTracker.logTeleopInit();
            disabledIterator.stop();
            compressor_.setClosedLoopControl(true);
            RobotState.getInstance().resetXY(Timer.getFPGATimestamp(), Pose2d.identity());
            enabledIterator.start();
            drive_.openLoop(new DriveSignal(0, 0));
            drive_.setRampMode(Drive.RampMode.LowLift);
            drive_.setHighGear();
            if (autoRunner_ != null) {
                autoRunner_.stop();
                autoRunner_ = null;
            }
            autoRunner_ = new AutoModeRunner();
            currentAutoMode_ = null;
            autoRunner_.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
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
     * This function is called at the start of test mode.
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
     * This function is called periodically during test mode.
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