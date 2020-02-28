package frc2020.auto.commands;

import frc2020.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

/**
 * This command tells the robot to shoot autonomously for a given
 * amount of time.
 * If we already are aiming at the target then we can go ahead and start shooting
 * otherwise this command will let the turret aim before shooting.
 */
public class Shoot implements Command {
    private static Shooter SHOOTER = Shooter.getInstance();

    private double shootStartTime_;
    private double shootTime_;
    private boolean needsToAim_;

    private Shooter.ShooterState endState = Shooter.ShooterState.Stowed;

    /**
     * Construct a shooting command with a given shoot time
     * @param shootTime The time to shoot for in sec
     */
    public Shoot(double shootTime) {
        this.shootTime_ = shootTime;
        this.shootStartTime_ = -1.0;
        this.needsToAim_ = false;
    }

    /**
     * Construct a shooter with a given shoot time and a state
     * to go to after it's done shooting.
     * @param shootTime The time to shoot for in sec
     * @param endState The end state to go to once done shooting
     */
    public Shoot(double shootTime, Shooter.ShooterState endState) {
        this(shootTime);
        this.endState = endState;
    }

    /**
     * Return if we are finished shooting. This is based on
     * weather or not we have been shooting for the given shoot
     * time.
     * @return Weather we are finished or not.
     */
    @Override
    public boolean isFinished() {
        if (shootStartTime_ >= 0.0) {
            double now = Timer.getFPGATimestamp();
            if (now - shootStartTime_ >= shootTime_) {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks to see weather or not we have already been aimed.
     * If we haven't the the turret aim before starting to shoot.
     */
    @Override
    public void update() {
        if (needsToAim_ || SHOOTER.getState() == Shooter.ShooterState.Aiming) {
            if (SHOOTER.hasTarget()) {
                SHOOTER.setState(Shooter.ShooterState.Shooting);
                shootStartTime_ = Timer.getFPGATimestamp();
                needsToAim_ = false;
            }
        }
    }

    /**
     * Set the shooter to the specified end sate once we are done.
     * (Default manual)
     */
    @Override
    public void done() {
        SHOOTER.setState(endState);
    }

    /**
     * Check weather or not we need to aim before starting the shooter.<p> If
     * we are in manual or stowed we defiantly need to aim. The other case we need
     * to aim in is if we are in aiming and we don't have a target. </p>
     * If none of these conditions are true then just go ahead and start
     * shooting.
     */
    @Override
    public void start() {
        var shooterState = SHOOTER.getState();
        needsToAim_ = shooterState == Shooter.ShooterState.Manual
                || shooterState == Shooter.ShooterState.Stowed
                || (shooterState == Shooter.ShooterState.Aiming && !SHOOTER.hasTarget());
        if (needsToAim_) {
            SHOOTER.setState(Shooter.ShooterState.Aiming);
        } else {
            SHOOTER.setState(Shooter.ShooterState.Shooting);
            shootStartTime_ = Timer.getFPGATimestamp();
        }
    }
}
