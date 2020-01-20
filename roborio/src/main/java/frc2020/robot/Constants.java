package frc2020.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * This class is masking the electrical diagram but in code.
 * It tells us where each component is plugged in so that we can
 * quickly update the entire robot for electrical wiring changes
 */
public class Constants {

    //CanBUS
    public final static int CAN_TIMEOUT = 50;
    public final static int CAN_TIMEOUT_LONG = 100;
    public final static int STATUS_FRAME_TIME = 50;

    // Drive Train Ports
    public final static int LEFT_MASTER_PORT = 1; // Spark Max
    public final static int LEFT_SLAVE_PORT = 2; // Spark Max

    public final static int RIGHT_MASTER_PORT = 3; // Spark Max
    public final static int RIGHT_SLAVE_PORT = 4; // Spark Max

    // Motor Controller Properties
    public final static double OPEN_LOOP_RAMP = 0.25; // seconds
    public final static double CLOSED_LOOP_RAMP = 0.25; // seconds
    public final static int STALL_LIMIT = 45; // Amps DC
    public final static int FREE_LIMIT = 40; // Amps DC

    //The Shifters (drive)
    public final static int SHIFT_FORWARD = 3;
    public final static int SHIFT_REVERSE = 4;

    // Drive Train Physical Properties
    // meters NOTE: This is the effective wheel diameter not the measured one
    public final static double WHEEL_DIAMETER = 0.1522414151; // meters
    public final static double WHEEL_RADIUS = WHEEL_DIAMETER / 2; // meters
    public final static double DRIVE_TRACK_WIDTH = 0.635; // meters
    public final static double TRACK_SCRUB_FACTOR = 1.062; // We use this to calculate effective wheel base
    public final static double ROBOT_LENGTH = 0.97155; //meters

    public final static double DRIVE_V_INTERCEPT = 0.189;  // Volts
    public final static double DRIVE_KV = 2.28;  // V per rad/s
    public final static double DRIVE_KA = 0.494; // V per rad/s^2

    // Drive Train Control Loop (VELOCITY)
    public final static double VELOCITY_HIGH_GEAR_KP = 0.0005; //2.28
    public final static double VELOCITY_HIGH_GEAR_KI = 0.0; //0.003
    public final static double VELOCITY_HIGH_GEAR_KD = 0.0; 
    public final static double VELOCITY_HIGH_GEAR_KF = 0.0;
    public final static int VELOCITY_HIGH_GEAR_I_ZONE = 300; //sensor units

    // The Joysticks
    public final static int LEFT_DRIVER_JOYSTICK_PORT = 0;
    public final static int RIGHT_DRIVER_JOYSTICK_PORT = 1;

    public final static int DRIVE_TOGGLE_SHIFT_BUTTON = 4;

    //CANcoder IDs
    public final static int LEFT_CAN_CODER_ID = 1;
    public final static int RIGHT_CAN_CODER_ID = 2;

    //Limelight
    public static final double LIMELIGHT_RES_X = 320.0;
    public static final double LIMELIGHT_RES_Y = 240.0;
    public static final double LIMELIGHT_HORIZONTAL_FOV = 59.6;
    public static final double LIMELIGHT_VERTICAL_FOV = 49.7;

    public static final double TARGET_HEIGHT = 2.496; // meters
    public static final double TARGET_NORMAL = 180.0; // degrees

    public static final double MAX_TRACK_DISTANCE = 0.2286; // meters
    public static final double MAX_TRACK_AGE = 2.5; // sec
    public static final double MAX_GOAL_TRACK_AGE_NOT_TRACKING = 0.1; // sec
    public static final double MAX_GOAL_TRACK_SMOOTHING_TIME = 0.5; // sec
    public static final double TRACK_STABILITY_WEIGHT = 0.0;
    public static final double TRACK_AGE_WEIGHT = 10.0;
    public static final double TRACK_SWITCHING_WEIGHT = 100.0;

    public static final double CAMERA_FRAME_RATE = 90.0;
    public static final double MIN_STABILITY = 0.5;

    
    // Defines the plane 1.0 unit away from the camera
    public static final double VERTICAL_PLANE_HEIGHT = 2.0 *
        Math.tan(Math.toRadians(LIMELIGHT_VERTICAL_FOV / 2.0)); 
    public static final double VERTICAL_PLANE_WIDTH = 2.0 *
        Math.tan(Math.toRadians(LIMELIGHT_HORIZONTAL_FOV / 2.0));
    public static final double IMAGE_CAPTURE_LATENCY = 11.0 / 1000.0; // seconds

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            final Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            final StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                final NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    final byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (final SocketException e) {
            e.printStackTrace();
        } catch (final NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
