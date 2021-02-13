package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;

public class FloodGate implements Subsystem {
  private static FloodGate instance_;

  private Climber climber_;

  public static FloodGate getInstance() {
    return (instance_ == null) ? instance_ = new FloodGate() : instance_;
  }

  private FloodGate() {
    climber_ = Climber.getInstance();
  }

  public void toggle() {
    climber_.toggleFloodGate();
  }

  public void extend() {
    climber_.deployFloodGate();
  }

  public void retract() {
    climber_.retractFloodGate();
  }

  @Override
  public void writePeriodicOutputs() {}

  @Override
  public void readPeriodicInputs() {}

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
  public void stop() {}

  @Override
  public void registerLoops(ILooper enableLooper) {}

  @Override
  public void outputTelemetry() {}
}