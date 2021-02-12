package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;

public class FloodGate implements Subsystem {
  private static FloodGate instance_;

  private DoubleSolenoid floodGateActuator_;
  private final static int FLOOD_GATE_ACTUATOR_PCM_ID = 0;
  private final static int FLOOD_GATE_ACTUATOR_FORWARD_PORT = 0;
  private final static int FLOOD_GATE_ACTUATOR_REVERSE_PORT = 1;
  private boolean isExtended_ = false;
  private boolean wantExtended_ = false;

  private final static DoubleSolenoid.Value RETRACTED_VALUE = Value.kForward;
  private final static DoubleSolenoid.Value EXTENDED_VALUE = Value.kReverse;

  public static FloodGate getInstance() {
    return (instance_ == null) ? instance_ = new FloodGate() : instance_;
  }

  private FloodGate() {
    floodGateActuator_ = new DoubleSolenoid(FLOOD_GATE_ACTUATOR_PCM_ID, FLOOD_GATE_ACTUATOR_FORWARD_PORT,
                                            FLOOD_GATE_ACTUATOR_REVERSE_PORT);
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
      floodGateActuator_.set(wantExtended_ ? EXTENDED_VALUE : RETRACTED_VALUE);
    }
  }

  @Override
  public void readPeriodicInputs() {
    isExtended_ = floodGateActuator_.get() == EXTENDED_VALUE;
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