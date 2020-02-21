package frc2020.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import frc2020.util.Logger;

/**
 * This class is masking the electrical diagram but in code.
 * It tells us where each component is plugged in so that we can
 * quickly update the entire robot for electrical wiring changes
 */
public class Constants {
    // Logger
    public final static double LOGGER_FLUSH_TIME = 2.0;
    private static Logger logger_ = Logger.getInstance();

    // Passive tests
    public final static double PASSIVE_TEST_TIME = 10.0;

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
    public final static int SHIFT_FORWARD = 6;
    public final static int SHIFT_REVERSE = 7;

    //Climber
    public final static int CLIMB_FORWARD = 2;
    public final static int CLIMB_REVERSE = 3;

    // Drive Train Physical Properties
    // meters NOTE: This is the effective wheel diameter not the measured one
    public final static double WHEEL_DIAMETER = 0.1522414151; // meters
    public final static double WHEEL_RADIUS = WHEEL_DIAMETER / 2; // meters
    public final static double DRIVE_TRACK_WIDTH = 0.635; // meters
    public final static double TRACK_SCRUB_FACTOR = 1.062; // We use this to calculate effective wheel base
    public final static double ROBOT_LENGTH = 0.97155; //meters
    public final static double ROBOT_WIDTH = 0.8382; //meters

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
    public final static int LEFT_SECONDARY_DRIVER_JOYSTICK_PORT = 3;
    public final static int RIGHT_SECONDARY_DRIVER_JOYSTICK_PORT = 4;
    public final static int AUTO_STEER_BUTTON = 2; // left driver
    public final static int AUTO_ALIGN_BUTTON = 3; // left driver
    public final static int MANUAL_CONTROL_BUTTON_1 = 7; //right second
    public final static int MANUAL_CONTROL_BUTTON_2 = 13; //right second
    public final static int MANUAL_FEEDER_INTAKE_HAT = 0; //right second
    public final static int MANUAL_FEEDER_OUTTAKE_HAT = 180; //right second
    public final static int MANUAL_CONTROL_PANEL_CLOCKWISE_HAT = 0; //left second
    public final static int MANUAL_CONTROL_PANEL_COUNTERCLOCKWISE_HAT = 180; //left second
    public final static int INTAKE_DEPLOY_TOGGLE = 3; //right driver
    public final static int INTAKE_OUTTAKE_BUTTON = 2; // right driver
    public final static int INTAKE_INTAKE_BUTTON = 1; // right driver trigger
    public final static int FLYWHEEL_SPIN_TOGGLE = 2; // right second

    public final static int DRIVE_TOGGLE_SHIFT_BUTTON = 4; // right driver

    //CANcoder IDs
    public final static int LEFT_CAN_CODER_ID = 1;
    public final static int RIGHT_CAN_CODER_ID = 2;

    // Limelight pipelines
    public final static int POWER_CELL_PIPELINE = 3;
    public final static int DRIVER_MODE_PIPELINE = 2;
    public final static int LOADING_STATION_PIPELINE = 4;

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
                        logger_.logError("Address doesn't exist or is not accessible");
                    }
                } else {
                    logger_.logError("Network Interface for the specified address is not found.");
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
