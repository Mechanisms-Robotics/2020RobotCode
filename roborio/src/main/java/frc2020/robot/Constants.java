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

    //The Shifters (drive)
    public final static int SHIFT_FORWARD = 4;
    public final static int SHIFT_REVERSE = 3;

    // Drive Train Physical Properties
    // meters NOTE: This is the effective wheel diameter not the measured one
    public final static double WHEEL_DIAMETER = 0.1522414151; // meters
    public final static double WHEEL_RADIUS = WHEEL_DIAMETER / 2; // meters
    public final static double DRIVE_TRACK_WIDTH = 0.635; // meters
    public final static double TRACK_SCRUB_FACTOR = 1.062; // We use this to calculate effective wheel base

    public final static double DRIVE_V_INTERCEPT = 0.799;  // Volts
    public final static double DRIVE_KV = 0.262;  // V per rad/s
    public final static double DRIVE_KA = 0.066; // V per rad/s^2

    // Drive Train Control Loop (VELOCITY)
    public final static double VELOCITY_HIGH_GEAR_KP = 1;
    public final static double VELOCITY_HIGH_GEAR_KI = 0.003;
    public final static double VELOCITY_HIGH_GEAR_KD = 0.0; 
    public final static double VELOCITY_HIGH_GEAR_KF = 0.0;
    public final static int VELOCITY_HIGH_GEAR_I_ZONE = 300; //sensor units

    // The Joysticks
    public final static int LEFT_DRIVER_JOYSTICK_PORT = 0;
    public final static int RIGHT_DRIVER_JOYSTICK_PORT = 1;

    public final static int DRIVE_TOGGLE_SHIFT_BUTTON = 4;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
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
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
