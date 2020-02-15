package frc2020.subsystems;

import frc2020.subsystems.Subsystem;
import frc2020.subsystems.Flywheel;
import frc2020.subsystems.Feeder;
import frc2020.subsystems.Hood;
import frc2020.subsystems.Turret;
import frc2020.util.*;
import frc2020.loops.ILooper;


public class Shooter {

    private static Shooter instance_; 

    private Flywheel flywheel_;
    private Feeder feeder_;
    private Hood hood_;
    private Turret turret_;

    private Logger logger_ = new Logger();

    public enum ShooterState {
        Manual,
        Stowed,
        Aiming,
        Shooting
    };

    private static ShooterState state_ = ShooterState.Stowed;

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
            // Handle logic
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
                logger_.logWarning("Invalid shooter state transition!");
                return false;
        }
    }

};