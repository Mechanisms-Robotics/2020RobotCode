package frc2020.loops;

import frc2020.robot.Robot;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which doesn't roll over
 */
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    private static String outFile;
    static {
        if (Robot.isReal()) {
            outFile = "/home/lvuser/crash_tracking.txt";
        } else {
            outFile = "C:\\Users\\Public\\test.txt";
        }
    }

    /**
    * Logs robot startup
    */
    public static void logRobotStartup() {
        logMarker("robot startup");
    }

    /**
    * Logs robot startup
    */
    public static void logRobotConstruction() {
        logMarker("robot construction");
    }

    /**
    * Logs robot init
    */
    public static void logRobotInit() {
        logMarker("robot init");
    }

    /**
    * Logs teleop init
    */
    public static void logTeleopInit() {
        logMarker("teleop init");
    }

    /**
    * Logs auto init
    */
    public static void logAutoInit() {
        logMarker("auto init");
    }

    /**
    * Logs disabled init
    */
    public static void logDisabledInit() {
        logMarker("disabled init");
    }

    /**
    * Logs throwable crash
    */
    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    /**
    * Writes UUID, string, and date
    */
    protected static void logMarker(String mark) {
        logMarker(mark, null);
    }

    /**
    * Writes UUID, string, date, and a stack trace if it was passed an exception
    */
    protected static void logMarker(String mark, Throwable nullableException) {

        try (PrintWriter writer = new PrintWriter(new FileWriter(outFile, true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
