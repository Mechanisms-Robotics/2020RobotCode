package frc2020.auto.modes;

import frc2020.auto.AutoMode;
import frc2020.auto.AutoModeEndedException;
import frc2020.auto.commands.*;

/**
 * Test auto mode for just trying out different paths, velocities, accelerations, etc
 */
public class RevTimedAuto extends AutoMode {

  @Override
  protected void routine() throws AutoModeEndedException {
    runCommand(new Shoot(4.0));
    runCommand(new TimedDrive(2.5, true));
  }
}