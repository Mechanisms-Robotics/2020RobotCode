package frc2020.auto.commands;

import frc2020.subsystems.Shooter;
import frc2020.subsystems.Shooter.ShooterState;

public class StowShooter implements Command {
  private static Shooter shooter_ = Shooter.getInstance();
  private boolean waitForStow;

  public StowShooter(boolean waitForStow) {
    this.waitForStow = waitForStow;
  }

  @Override
  public boolean isFinished() { return waitForStow ? shooter_.getState() == ShooterState.Stowed : true; }

  @Override
  public void update() {}

  @Override
  public void done() {}

  @Override
  public void start() {
    shooter_.setState(ShooterState.Stowed);
  }
}
