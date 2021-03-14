package frc2020.auto.commands;

import frc2020.subsystems.Shooter;
import frc2020.subsystems.Shooter.ShooterState;

public class ShootWhileDriving implements Command {
  private static Shooter shooter_ = Shooter.getInstance();

  private boolean needsToAim_;

  @Override
  public boolean isFinished() { return shooter_.getState() == ShooterState.Shooting; }

  @Override
  public void update() {
    if (needsToAim_ && shooter_.getState() == Shooter.ShooterState.Aiming) {
      if (shooter_.hasTarget()) {
        shooter_.setState(Shooter.ShooterState.Shooting);
        needsToAim_ = false;
      }
    }
  }

  @Override
  public void done() {}

  @Override
  public void start() {
    var shooterState = shooter_.getState();
    needsToAim_ = shooterState == Shooter.ShooterState.Manual
        || shooterState == Shooter.ShooterState.Stowed
        || (shooterState == Shooter.ShooterState.Aiming && !shooter_.hasTarget());
    if (needsToAim_) {
      shooter_.setState(Shooter.ShooterState.Aiming);
    } else {
      shooter_.setState(Shooter.ShooterState.Shooting);
    }
  }
}
