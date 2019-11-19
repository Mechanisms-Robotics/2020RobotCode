package frc2020.util;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchedBoolean {
    private boolean last_ = false;

    public boolean update(boolean newValue) {
        boolean ret = false;
        if (newValue && !last_) {
            ret = true;
        }
        last_ = newValue;
        return ret;
    }
}