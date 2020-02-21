package frc2020.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.subsystems.Subsystem;
import frc2020.subsystems.Flywheel;
import frc2020.subsystems.Feeder;
import frc2020.subsystems.Hood;
import frc2020.subsystems.Turret;
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

    private Logger logger_ = new Logger();
    private String logName = "Shooter";

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
    }

    public void setState(ShooterState desiredState) {
        if (isValidTransition(desiredState)) {
            wantedState_ = desiredState;
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
                    case Stowed:
                        handleStowedTransition();
                    case Aiming:
                        handleAimingTransition();
                    case Shooting:
                        handleShootingTransition();
                    default:
                        logger_.logWarning("Invalid wanted state: " + state_.toString(), logName);
                }
            }
        }

        @Override
        public void end() {
        }
        
    };

    private void handleManual() {
        // Nothing to do for now
    }

    private void handleStowed() {
        feeder_.setState(FeederState.INTAKING);
    }

    private void handleAiming() {

        // TODO: If not have target seek target

        double azimuth = limelight_.getTargetReading().azimuth;

        turret_.setRelativePosition(Rotation2d.fromDegrees(azimuth));

        feeder_.setState(FeederState.PRIMING);
    }

    private void handleShooting() {

        double azimuth = limelight_.getTargetReading().azimuth;

        turret_.setRelativePosition(Rotation2d.fromDegrees(azimuth));

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

        turret_.setAbsolutePosition(Rotation2d.fromDegrees(0.0));

        limelight_.setLed(Limelight.LedMode.OFF);

        if (!flywheel_.isStopped()) {
            return;
        }

        hood_.stowHood();

        if (!hood_.isStowed()) {
            return;
        }

        state_ = ShooterState.Stowed;
    }

    private void handleAimingTransition () {

        flywheel_.stop();

        limelight_.setLed(Limelight.LedMode.ON);

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
    }

}