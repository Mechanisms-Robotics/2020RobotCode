package frc2020.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * This class is a thin wrapper around the CANVictor that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Victor flushes the Tx buffer on every set call).
 */
public class LazyVictorSPX extends VictorSPX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    /**
     * Constructs VictorSPX object
     * @param deviceNumber ID of Victor
     */
    public LazyVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }

    /**
     * Getter for the last set value on the victor
     * @return previously set value
     */
    public double getLastSet() {
        return mLastSet;
    }

    /**
     * See super implementation but basically sets outputs
     * 
     * @param mode control mode. See super for types
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
