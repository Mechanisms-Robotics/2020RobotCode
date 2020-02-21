package frc2020.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.subsystems.Subsystem;
import frc2020.subsystems.Flywheel;
import frc2020.subsystems.Feeder;
import frc2020.subsystems.Hood;
import frc2020.subsystems.Turret;
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

    public Shooter(Limelight ll) {
        limelight_ = ll;
        flywheel_ = Flywheel.getInstance();
        feeder_ = Feeder.getInstance();
        hood_ = Hood.getInstance();
        turret_ = Turret.getInstance();
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
        // TODO Auto-generated method stub

    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean runPassiveTests() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean runActiveTests() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub

    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

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
                // TODO: Implement periodic functions
                switch (state_) {
                    case Manual:
                    case Stowed:
                    case Aiming:
                    case Shooting:
                    default:
                        logger_.logWarning("Invalid state: " + state_.toString(), logName);
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

    private void handleManualTransition() {
        flywheel_.stop();
        feeder_.stop();
        hood_.stop();
        turret_.stop();
        state_ = ShooterState.Manual;
    }

    private void handleStowedTransition() {
        flywheel_.stop();
        if (!flywheel_.atDemand()) {
            return;
        }

        // TODO: Set feeder state to intaking

        turret_.setAbsolutePosition(Rotation2d.fromDegrees(0.0));
        limelight_.setLed(Limelight.LedMode.OFF);

        if (!turret_.atDemand()) {
            return;
        }

        hood_.stowHood();

        if (!hood_.isStowed()) {
            return;
        }

        state_ = ShooterState.Stowed;
    }

    private void handleAimingTransition () {

        hood_.stowHood();

        if (!hood_.isStowed()) {
            return;
        }

        limelight_.setLed(Limelight.LedMode.ON);

        // TODO: Set turret state to auto

        // TODO: Prime feeder

        // TODO: Wait for those to be complete

        state_ = ShooterState.Aiming;
    }

    private void handleShootingTransition() {

        hood_.deployHood();

        if (!hood_.isDeployed()) {
            return;
        }

        flywheel_.spinFlywheel();

        if (!flywheel_.atDemand()) {
            return;
        }

        state_ = ShooterState.Shooting;
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

}