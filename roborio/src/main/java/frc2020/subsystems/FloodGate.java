package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;

public class FloodGate implements Subsystem {
  private static FloodGate instance_;

  private Climber climber_;

  private boolean isExtended_ = false;
  private boolean wantExtended_ = false;

  public static FloodGate getInstance() {
    return (instance_ == null) ? instance_ = new FloodGate() : instance_;
  }

  private FloodGate() {
    climber_ = Climber.getInstance();
  }

  public void toggle() {
    wantExtended_ = !wantExtended_;
  }

  public void extend() {
    wantExtended_ = true;
  }

  public void retract() {
    wantExtended_ = false;
  }

  public boolean isExtended() {
    return isExtended_;
  }

  @Override
  public void writePeriodicOutputs() {
    if (wantExtended_ != isExtended_) {
      if (wantExtended_) {
        climber_.lockWinch();
      } else {
        climber_.unlockWinch();
      }
    }
  }

  @Override
  public void readPeriodicInputs() {
    isExtended_ = climber_.isLocked();
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
  public void zeroSensors() {}

  @Override
  public void stop() {

  }

  @Override
  public void registerLoops(ILooper enableLooper) {}

  @Override
  public void outputTelemetry() {}
}