package frc2020.util;

import edu.wpi.first.wpilibj.DriverStation;

public class Logger {

    private enum Level {
        Debug(3),
        Info(2),
        Waring(1),
        Error(0);

        private int level_int_;
        private Level(int level_int) {
            level_int_ = level_int;
        }

        public int getValue() {
            return level_int_;
        }
    }

    private static Level LOGGER_LEVEL = Level.Debug;

    public static void logDebug(String msg) {
        if (LOGGER_LEVEL.getValue() >= Level.Debug.getValue()) {
            System.out.println("DEBUG " + msg);
        }
    }

    public static void logInfo(String msg) {
        if (LOGGER_LEVEL.getValue() >= Level.Info.getValue()) {
            System.out.println("INFO " + msg);
        }
    }

    public static void logWaring(String msg) {
        if (LOGGER_LEVEL.getValue() >= Level.Waring.getValue()) {
            DriverStation.reportWarning(msg, false);
        }
    }
    
    public static void logError(String msg) {
        if (LOGGER_LEVEL.getValue() >= Level.Error.getValue()) {
            DriverStation.reportError(msg, false);
        }
    }
}