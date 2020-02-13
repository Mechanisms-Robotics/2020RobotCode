package frc2020.util;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
    protected double mLeftMotor;
    protected double mRightMotor;
    protected boolean mBrakeMode;

    /**
     * demand from 0 to 1
     */
    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    /**
     * demand from 0 to 1
     * @param brakeMode tells whether or not motors are locked in place or 
     * free to move
     */
    public DriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    /**
     * @param speeds Easier to determine speeds for a differential drive train
     */
    public DriveSignal(DifferentialDriveWheelSpeeds speeds, boolean brakeMode) {
        mLeftMotor = speeds.leftMetersPerSecond;
        mRightMotor = speeds.rightMetersPerSecond;
        mBrakeMode = brakeMode;
    }

    /**
     * Neutral mode so easy to push around while robot is disabled
     */
    public static DriveSignal NEUTRAL = new DriveSignal(0, 0);

    /**
     * Brake mode so robot comes to a hard stop doesn't move any further
     */
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}