package frc2020.subsystems;

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
    };

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
            // TODO Auto-generated method stub

        }

        @Override
        public void run() {
            // TODO Auto-generated method stub
            if (wantedState_ == state_) {
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
                    case Stowed:
                    case Aiming:
                    case Shooting:
                    default:
                        logger_.logWarning("Invalid wanted state: " + state_.toString(), logName);
                }
            }
        }

        @Override
        public void end() {
            // TODO Auto-generated method stub

        }
        
    };

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

};