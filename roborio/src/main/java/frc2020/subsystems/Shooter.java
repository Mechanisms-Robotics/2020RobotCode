package frc2020.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private Logger logger_ = Logger.getInstance();
    private String logName = "Shooter";


    private final static double TURRET_SEEKING_DUTY_CYCLE = 0.1; // duty cycle
    private final static double TURRET_SEEKING_DELTA_ANGLE = 5.0; // degrees

    private double startingPosition = 0.0;
    private double turretSeekPower_ = TURRET_SEEKING_DUTY_CYCLE;
    private boolean hasStartedSeeking_ = false;

    private LatchedBoolean seekTurretLatch_;

    public enum ShooterState {
        Manual,
        Stowed,
        Aiming,
        Shooting
    }

    private static ShooterState state_ = ShooterState.Stowed;
    private static ShooterState wantedState_ = ShooterState.Stowed;

    public static Shooter getInstance() {
        return (instance_ == null) ? instance_ = new Shooter() : instance_;
    }

    public Shooter() {
        flywheel_ = Flywheel.getInstance();
        feeder_ = Feeder.getInstance();
        hood_ = Hood.getInstance();
        turret_ = Turret.getInstance();
        seekTurretLatch_ = new LatchedBoolean();
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
            logger_.logInfo("Transitioning from " + state_ + " to " + desiredState, logName);
            wantedState_ = desiredState;
         } else {
            logger_.logWarning("Transitioning from " + state_ + " to " + desiredState + " IS INVALID", logName);
            wantedState_ = state_;
        }
    }

    public void setLimelight(Limelight ll) {
        limelight_ = ll;
    }

    private boolean isValidTransition(ShooterState desiredState) {

        switch (state_) {
            case Manual:
                return desiredState == ShooterState.Stowed;
            case Stowed:
                return (desiredState == ShooterState.Manual) || (desiredState == ShooterState.Aiming);
            case Aiming:
                return (desiredState == ShooterState.Shooting) || (desiredState == ShooterState.Stowed);
            case Shooting:
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
                    case Aiming:
                        handleAiming();
                        break;
                    case Shooting:
                        handleShooting();
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
                    case Aiming:
                        handleAimingTransition();
                        break;
                    case Shooting:
                        handleShootingTransition();
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

    private void handleManual() {
        feeder_.setState(FeederState.MANUAL);
    }

    private void handleStowed() {
        feeder_.setState(FeederState.INTAKING);
    }

    private void handleAiming() {
        if (limelight_.getTargetReading().hasConfidentTarget()) {
            /*double azimuth = limelight_.getTargetReading().azimuth;
            turret_.setRelativeRotation(Rotation2d.fromDegrees(azimuth));*/

            hasStartedSeeking_ = false;
        } else {
            //seekTurret();
        }
    }

    private void handleShooting() {

        double azimuth = limelight_.getTargetReading().azimuth;

        turret_.setRelativeRotation(Rotation2d.fromDegrees(azimuth));

        //TODO: Set hood angle automatically

        feeder_.runFeeder(false);

    }

    private void handleManualTransition() {
        flywheel_.stop();
        feeder_.stop();
        hood_.stop();
        turret_.stop();
        state_ = ShooterState.Manual;
    }

    private void handleStowedTransition() {
        flywheel_.stop();

        turret_.setAbsoluteRotation(Rotation2d.fromDegrees(0.0));

        limelight_.setLed(Limelight.LedMode.OFF);

        hood_.setToStowPosition();

        if (!flywheel_.isStopped()) {
            logger_.logDebug("Waiting for flywheel to stop!", logName);
            return;
        }

        logger_.logDebug("Flywheel stopped!", logName);

        hood_.stowHood();

        if (!hood_.isStowed()) {
            return;
        }

        logger_.logDebug("Hood stowed!", logName);

        state_ = ShooterState.Stowed;

        logger_.logDebug("Transition successful!", logName);
    }

    private void handleAimingTransition () {

        hasStartedSeeking_ = false;

        flywheel_.stop();

        limelight_.setLed(Limelight.LedMode.ON);

        hood_.setToStowPosition();

        // TODO: Aim turret in ball park

        feeder_.setState(FeederState.PRIMING);

        if (!feeder_.isPrimed()) {
            return;
        }

        if (!flywheel_.isStopped()) {
            return;
        }

        hood_.stowHood();

        if (!hood_.isStowed()) {
            return;
        }

        state_ = ShooterState.Aiming;
    }

    private void handleShootingTransition() {

        hood_.deployHood();

        if (!hood_.isDeployed()) {
            return;
        }

        // TODO: Set hood angle automatically

        flywheel_.spinFlywheel();

        if (!flywheel_.atDemand()) {
            return;
        }

        state_ = ShooterState.Shooting;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Shooter state: ", state_.toString());
        SmartDashboard.putString("Wanted state: ", wantedState_.toString());
    }

}
