package frc2020.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonSRX extends TalonSRX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    /**
     * Constructs TalonSRX object
     * @param deviceNumber ID of Talon
     */
    public LazyTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    /**
     * Getter for the last set value for the talon
     * 
     * @return previously set value
     */
    public double getLastSet() {
        return mLastSet;
    }

    /**
     * See super implementation. Basically just sets the talon's outputs
     * 
     * @param mode Control mode to set to the talon. See super for types
     * @param value setpoint value
     */
    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }
}