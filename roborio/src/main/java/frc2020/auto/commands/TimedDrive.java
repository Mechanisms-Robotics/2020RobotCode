package frc2020.auto.commands;

import frc2020.subsystems.Drive;
import frc2020.subsystems.Shooter;
import frc2020.subsystems.Shooter.ShooterState;
import edu.wpi.first.wpilibj.Timer;
import frc2020.util.DriveSignal;


public class TimedDrive implements Command {

  private double time = 0.0;
  private Timer timer = new Timer();
  private Drive drive = Drive.getInstance();
  private boolean reversed = false;

  public TimedDrive(double time) {
    this.time = time;
  }

  public TimedDrive(double time, boolean reversed) {
    this.time = time;
    this.reversed = reversed;
  }

  @Override
  public boolean isFinished() { return timer.hasElapsed(time); }

  @Override
  public void update() {}

  @Override
  public void done() { drive.driveVelocity(new DriveSignal(0.0, 0.0));}

  @Override
  public void start() {
    timer.start();
    if (!reversed) {
      drive.driveVelocity(new DriveSignal(0.5, 0.5));
    } else {
      drive.driveVelocity(new DriveSignal(-0.5, -0.5));
    }

  }
}