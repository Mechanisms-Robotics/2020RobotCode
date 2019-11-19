package frc2020.util.drivers;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Some useful utilities for TalonSRXs
 */
public class TalonSRXUtil {
    /**
     * Checks the specified error code
     *
     * @param errorCode The error code returned
     * @param message   The message to print out
     */
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}
