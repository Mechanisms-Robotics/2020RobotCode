package frc2020.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc2020.robot.Constants;
import frc2020.subsystems.Feeder.FeederState;
import frc2020.util.*;
import frc2020.loops.ILooper;
import frc2020.loops.Loop;


public class Shooter implements Subsystem {

    private static Shooter instance_;

    private Limelight limelight_ = null;

    private Flywheel flywheel_;
    private Feeder feeder_;
    private Hood hood_;
    private Turret turret_;
    private FloodGate floodGate_;
    private ControlPanel panelArm_;

    private Logger logger_ = Logger.getInstance();
    private String logName = "Shooter";

    private final static double FAR_FEEDER_DISTANCE = 5.99; // meters, should be same as MIDDLE_TRENCH

    // TOOD: Make sure seeking is good with the increase in turrent angle
    private final static double TURRET_SEEKING_DUTY_CYCLE = 0.07; // duty cycle
    private final static double TURRET_SEEKING_DELTA_ANGLE = 5.0; // degrees

    private final static double TRENCH_HOOD_POSITION = 3.476; // units
    private final static int POWER_PORT_SPEED = 4500;

    private double startingPosition = 0.0;
    private double turretSeekPower_ = TURRET_SEEKING_DUTY_CYCLE;
    private boolean hasStartedSeeking_ = false;
    private boolean overrideFeeder_ = false;

    private LatchedBoolean seekTurretLatch_;

    private Rotation2d safeTurretPosition = Rotation2d.fromDegrees(-60.0);

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodAngleRangeInterpolator;

    public enum ShooterState {
        Manual,
        Stowed,
        PowerPort,
        Trench,
        Aiming,
        Shooting,
        Spinning
    }

    private static ShooterState state_ = ShooterState.Stowed;
    private static ShooterState wantedState_ = ShooterState.Stowed;

    private final static double SPINNER_STOW_TIME = 1.0; // sec
    private Timer spinnerStowTimer = new Timer();
    private boolean spinnerStowing = false;

    public static Shooter getInstance() {
        return (instance_ == null) ? instance_ = new Shooter() : instance_;
    }

    public Shooter() {
        flywheel_ = Flywheel.getInstance();
        feeder_ = Feeder.getInstance();
        hood_ = Hood.getInstance();
        turret_ = Turret.getInstance();
        floodGate_ = FloodGate.getInstance();
        panelArm_ = ControlPanel.getInstance();


        seekTurretLatch_ = new LatchedBoolean();
        hoodAngleRangeInterpolator = new InterpolatingTreeMap<>(100);
        loadHoodRangeAngleValues();
    }

    public synchronized ShooterState getState() {
        return state_;
    }

    public synchronized ShooterState getWantedState() { return wantedState_; }

    public synchronized void setState(ShooterState desiredState) {
        if (state_ == desiredState) {
            // logger_.logDebug("Already in " + desiredState, logName);
            return;
        }
        if (isValidTransition(desiredState)) {
            //logger_.logDebug("Transitioning from " + state_ + " to " + desiredState, logName); // TODO: fix spamming
            wantedState_ = desiredState;
        } else {
            logger_.logWarning("Transitioning from " + state_ + " to " + desiredState + " IS INVALID", logName);
            wantedState_ = state_;
        }
    }

    public void setLimelight(Limelight ll) {
        limelight_ = ll;
    }

    public synchronized void setOverrideFeeder(boolean overrideFeeder) {
        overrideFeeder_ = overrideFeeder;
    }

    private boolean isValidTransition(ShooterState desiredState) {
        if (desiredState == ShooterState.Manual) {
            return true;
        }

        switch (state_) {
            case Manual:
                return desiredState == ShooterState.Stowed;
            case Stowed:
                return desiredState == ShooterState.Aiming ||
                    desiredState == ShooterState.Shooting ||
                    desiredState == ShooterState.PowerPort ||
                    desiredState == ShooterState.Trench ||
                    desiredState == ShooterState.Spinning;
            case PowerPort:
                return desiredState == ShooterState.Stowed;
            case Trench:
                return desiredState == ShooterState.Stowed;
            case Aiming:
                return desiredState == ShooterState.Shooting ||
                    desiredState == ShooterState.Stowed ||
                    desiredState == ShooterState.PowerPort;
            case Shooting:
                return desiredState == ShooterState.Stowed;
            case Spinning:
                return desiredState == ShooterState.Stowed;
            default:
                logger_.logWarning("Invalid shooter state transition from " +
                    state_.toString() + " to " + desiredState.toString() + "!", logName);
                return false;
        }
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public void readPeriodicInputs() {
    }

    @Override
    public boolean runPassiveTests() {
        return true;
    }

    @Override
    public boolean runActiveTests() {
        return true;
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void stop() {
    }


    @Override
    public void registerLoops(ILooper enabledLooper) {
        enabledLooper.register(shooterLoop);
    }

    private final Loop shooterLoop = new Loop() {

        @Override
        public void init() {
        }

        @Override
        public void run() {
            if (wantedState_ == state_) {
                switch (state_) {
                    case Manual:
                        handleManual();
                        break;
                    case Stowed:
                        handleStowed();
                        break;
                    case PowerPort:
                        handlePowerPort();
                        break;
                    case Trench:
                        handleTrench();
                        break;
                    case Aiming:
                        handleAiming();
                        break;
                    case Shooting:
                        handleShooting();
                        break;
                    case Spinning:
                        handleSpinning();
                        break;
                    default:
                        logger_.logWarning("Invalid state: " + state_.toString(), logName);
                        break;
                }
            } else {
                switch (wantedState_) {
                    case Manual:
                        handleManualTransition();
                        break;
                    case Stowed:
                        handleStowedTransition();
                        break;
                    case PowerPort:
                        handlePowerPortTransition();
                        break;
                    case Trench:
                        handleTrenchTransition();
                        break;
                    case Aiming:
                        handleAimingTransition();
                        break;
                    case Shooting:
                        handleShootingTransition();
                        break;
                    case Spinning:
                        handleSpinningTransition();
                        break;
                    default:
                        logger_.logWarning("Invalid wanted state: " + state_.toString(), logName);
                        break;
                }
            }
        }

        @Override
        public void end() {
        }

    };

    private void seekTurret() {
        if (!hasStartedSeeking_) {
            startingPosition = turret_.getPosition();
            hasStartedSeeking_ = true;
        }

        turret_.setOpenLoop(turretSeekPower_);

        boolean outsideOfDelta = seekTurretLatch_.update(Math.abs(turret_.getPosition()-startingPosition) >= TURRET_SEEKING_DELTA_ANGLE
            || (turret_.atForwardLimit() || turret_.atReverseLimit()));

        if (outsideOfDelta) {
            turretSeekPower_ *= -1.0;
        }
    }

    private void autoTurret() {
        double azimuth = -limelight_.getTargetReading().azimuth;
        turret_.setRelativeRotation(Rotation2d.fromDegrees(azimuth));
    }

    private void autoHood() {
        double range = limelight_.getTargetReading().range;
        double setpoint = hoodAngleRangeInterpolator.getInterpolated(new InterpolatingDouble(range)).value;
        hood_.setSmartPosition(setpoint);
    }

    private boolean handleOverrideFeeder() {
        if (!overrideFeeder_) {
            return true;
        }
        feeder_.setState(FeederState.MANUAL);
        return false;
    }

    private void loadHoodRangeAngleValues() {
        // (range, angle)
        if (Constants.IS_COMP_BOT) {
            // These are the values before 3 March tuning
//            hoodAngleRangeInterpolator.put(new InterpolatingDouble(2.22), new InterpolatingDouble(2.12));
//            hoodAngleRangeInterpolator.put(new InterpolatingDouble(3.04), new InterpolatingDouble(2.69));
//            hoodAngleRangeInterpolator.put(new InterpolatingDouble(4.12), new InterpolatingDouble(3.07));
//            hoodAngleRangeInterpolator.put(new InterpolatingDouble(5.18), new InterpolatingDouble(3.27));
//            hoodAngleRangeInterpolator.put(new InterpolatingDouble(6.26), new InterpolatingDouble(3.28));
//            hoodAngleRangeInterpolator.put(new InterpolatingDouble(7.17), new InterpolatingDouble(3.30));
//            hoodAngleRangeInterpolator.put(new InterpolatingDouble(8.70), new InterpolatingDouble(2.60));

            // TUNABLES

            // position robot just OUTSIDE of trench
            final var BEGINNING_OF_TRENCH = new InterpolatingDouble(4.46);
            final var BEGINNING_OF_TRENCH_HOOD = new InterpolatingDouble(3.6); // decrease to shoot higher

            // position halfway between the two
            final var MIDDLE_OF_TRENCH = new InterpolatingDouble(5.99); // (calculated, not measured)
            final var MIDDLE_OF_TRENCH_HOOD = new InterpolatingDouble(2.1); // decrease to shoot higher

            // position robot as far BACK in trench as possible
            final var END_OF_TRENCH = new InterpolatingDouble(7.51);
            final var END_OF_TRENCH_HOOD = new InterpolatingDouble(0.3); // decrease to shoot higher

            // position robot BEYOND trench in front of power port
            final var BEYOND_TRENCH = new InterpolatingDouble(10.0);
            final var BEYOND_TRENCH_HOOD = new InterpolatingDouble(2.78); // decrease to shoot a bit higher

            // Check polarity of these three points because there was no comment on them.
            hoodAngleRangeInterpolator.put(new InterpolatingDouble(2.22), new InterpolatingDouble(2.35));
            hoodAngleRangeInterpolator.put(new InterpolatingDouble(3.04), new InterpolatingDouble(2.4));
            hoodAngleRangeInterpolator.put(new InterpolatingDouble(4.12), new InterpolatingDouble(2.7));
            hoodAngleRangeInterpolator.put(BEGINNING_OF_TRENCH, BEGINNING_OF_TRENCH_HOOD);
            hoodAngleRangeInterpolator.put(MIDDLE_OF_TRENCH, MIDDLE_OF_TRENCH_HOOD);
            hoodAngleRangeInterpolator.put(END_OF_TRENCH, END_OF_TRENCH_HOOD);
            hoodAngleRangeInterpolator.put(BEYOND_TRENCH, BEYOND_TRENCH_HOOD);
        }
    }

    private void handleManual() {
        if (handleOverrideFeeder()) {
            feeder_.setState(FeederState.MANUAL);
        }
    }

    private void handleStowed() {
        if (handleOverrideFeeder()) {
            feeder_.setState(FeederState.INTAKING);
            //feeder_.setState(FeederState.MANUAL);
        }
    }

    private void handlePowerPort() {
        if (handleOverrideFeeder()) {
            feeder_.runFeeder(Feeder.INTAKE_SPEED);
        }
    }

    private void handleTrench() {
        if (handleOverrideFeeder()) {
            feeder_.setState(FeederState.SHOOTING);
        }
    }

    private void handleAiming() {
        if (limelight_.getTargetReading().hasConfidentTarget()) {
            autoTurret();

            hasStartedSeeking_ = false;
        } else {
            seekTurret();
        }
    }

    private void handleShooting() {

        if (limelight_.getTargetReading().hasConfidentTarget()) {
            autoTurret();
            autoHood();
        }

        if (handleOverrideFeeder()) {
            feeder_.shootFeeder();
        }
    }

    private void handleSpinning() {
        if (handleOverrideFeeder()) {
            feeder_.setState(FeederState.INTAKING);
        }
    }

    private void handleManualTransition() {

        if (handleSpinnerStow()) {
            return;
        }

        flywheel_.stop();
        feeder_.stop();
        hood_.stop();
        turret_.stop();
        floodGate_.extend();
        limelight_.setLed(Limelight.LedMode.ON);
        state_ = ShooterState.Manual;
    }

    private void handleStowedTransition() {

        if (handleSpinnerStow()) {
            return;
        }

        floodGate_.extend();
        flywheel_.stop();
        feeder_.setState(FeederState.INTAKING);

        turret_.setAbsoluteRotation(Rotation2d.fromDegrees(0.0));

        limelight_.setLed(Limelight.LedMode.OFF);

        hood_.setToStowPosition();

        if (!turret_.atDemand() || !hood_.atDemand()) {
            if (hood_.atDemand()) {
                hood_.stowHood();
            }
            return;
        }

        hood_.stowHood();

        if (!hood_.isStowed()) {
            return;
        }

        state_ = ShooterState.Stowed;
    }

    private void handlePowerPortTransition() {

        if (handleSpinnerStow()) {
            return;
        }

        floodGate_.retract();
        feeder_.setState(FeederState.PRIMING);

        turret_.setAbsoluteRotation(Rotation2d.fromDegrees(0.0));

        hood_.deployHood();
        flywheel_.spinFlywheel(POWER_PORT_SPEED);

        if(!feeder_.isPrimed() || !turret_.atDemand() ||
           !hood_.isDeployed() || !flywheel_.atVelocity(POWER_PORT_SPEED)) {
            if (hood_.isDeployed()) {
                hood_.setToStowPosition();
            }
            return;
        }

        hood_.setToStowPosition();

        if(!hood_.atDemand()) {
            return;
        }

        state_ = ShooterState.PowerPort;
    }

    private void handleTrenchTransition() {

        if (handleSpinnerStow()) {
            return;
        }

        hood_.deployHood();
        floodGate_.retract();
        feeder_.setState(FeederState.PRIMING);

        turret_.setAbsoluteRotation(Rotation2d.fromDegrees(0.0));

        flywheel_.spinFlywheel();

        if(!hood_.isDeployed() || !feeder_.isPrimed() ||
           !turret_.atDemand() || !flywheel_.upToSpeed()) {
            if (hood_.isDeployed()) {
                hood_.setSmartPosition(TRENCH_HOOD_POSITION);
            }
            return;
        }

        hood_.setSmartPosition(TRENCH_HOOD_POSITION);

        if(!hood_.atDemand()) {
            return;
        }

        state_ = ShooterState.Trench;
    }

    private void handleAimingTransition () {

        if (handleSpinnerStow()) {
            return;
        }

        floodGate_.extend();

        hasStartedSeeking_ = false;

        limelight_.setLed(Limelight.LedMode.PIPELINE);

        flywheel_.spinFlywheel();

        if (wantedState_ != ShooterState.Shooting) {
            hood_.setToStowPosition();

            if (!hood_.atDemand()) {
                return;
            }

            hood_.stowHood();

            if (!hood_.isStowed()) {
                return;
            }
        }

        state_ = ShooterState.Aiming;
    }

    private void handleShootingTransition() {
        if (handleSpinnerStow()) {
            return;
        }

        if (state_ == ShooterState.Stowed) {
            handleAimingTransition();
            return;
        }

        autoTurret();

        hood_.deployHood();
        floodGate_.retract();
        feeder_.setState(FeederState.PRIMING);

        if (!flywheel_.isSpinningUp()) {
            flywheel_.spinFlywheel();
        }

        if (!hood_.isDeployed() || !feeder_.isPrimed() || !flywheel_.upToSpeed()) {
            if (hood_.isDeployed()) {
                autoHood();
            }
            return;
        }

        autoHood();

        if (!hood_.atDemand()) {
            return;
        }

        limelight_.setLed(Limelight.LedMode.PIPELINE);

        state_ = ShooterState.Shooting;
    }

    private void handleSpinningTransition() {

        turret_.setAbsoluteRotation(safeTurretPosition);

        if (!turret_.atDemand()) {
            return;
        }

        panelArm_.deployPanelArm();

        state_ = ShooterState.Spinning;
    }

    public synchronized void handleReenable() {
        switch (state_) {
            case Manual:
                wantedState_ = ShooterState.Stowed;
                return;
            case Stowed:
                state_ = ShooterState.Manual; // To force transition
                wantedState_ = ShooterState.Stowed;
                return;
            case Aiming:
                state_ = ShooterState.Stowed; // To force transition
                wantedState_ = ShooterState.Aiming;
                return;
            case Shooting:
                state_ = ShooterState.Stowed; // To force transition
                wantedState_ = ShooterState.Aiming; // For safety precautions
                return;
            case Spinning:
                state_ = ShooterState.Spinning;
                wantedState_ = ShooterState.Stowed;
            default:
                logger_.logWarning("Invalid state on re-enable", logName);
        }
    }

    public synchronized boolean hasTarget() {
        if (limelight_ == null) {
            return false;
        } else {
            return limelight_.getTargetReading().hasConfidentTarget();
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Shooter state: ", state_.toString());
        SmartDashboard.putString("Wanted state: ", wantedState_.toString());
    }

    // Retruns true if the spinner is still stowing and false
    // if the spinner is safly stowed.
    private boolean handleSpinnerStow() {
        if (panelArm_.isDeployed()) {
            spinnerStowing = true;
            spinnerStowTimer.stop();
            spinnerStowTimer.reset();
            spinnerStowTimer.start();
            panelArm_.stowPanelArm();
        }
        if (spinnerStowing) {
            if (spinnerStowTimer.get() < SPINNER_STOW_TIME) {
                return true;
            }
        }
        spinnerStowTimer.stop();
        spinnerStowing = false;
        return false;
    }
}