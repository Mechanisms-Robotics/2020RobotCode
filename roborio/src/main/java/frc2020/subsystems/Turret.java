package frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.loops.ILooper;

public class Turret extends SingleMotorSubsystem {

    // TODO: Measure physical position of limits.
    private final static double LEFT_LIMIT_POS_POSITIVE = 140;
    private final static double RIGHT_LIMIT_POS_NEGATIVE = -140;
    private final static double LEFT_LIMIT_POS_NEGATIVE = LEFT_LIMIT_POS_POSITIVE - 360;
    private final static double RIGHT_LIMIT_POS_POSITIVE = RIGHT_LIMIT_POS_NEGATIVE + 360;

    private final static Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(180);

    private boolean leftLimit = false;
    private boolean rightLimit = false;

    @Override
    protected boolean handleZeroing() {
        if (leftLimit) {
            if (getPosition() >= 0) {
                encoder.setPosition(LEFT_LIMIT_POS_POSITIVE);
            } else {
                encoder.setPosition(LEFT_LIMIT_POS_NEGATIVE);
            }
            return true;
        } else if (rightLimit) {
            if (getPosition() >= 0) {
                encoder.setPosition(RIGHT_LIMIT_POS_POSITIVE);
            } else {
                encoder.setPosition(RIGHT_LIMIT_POS_NEGATIVE);
            }
            return true;
        }
        return false;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        // TODO: Set left and right limit
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected boolean atReverseLimit() {
        // This is a safety. If the robot starts with the turret pointed in the
        // wrong direction, this will prevent us from ripping out the chain.
        if (!hasBeenZeroed) {
            return io_.velocity <= 0.0 && leftLimit;
        }

        // If were are here, the robot has been zeroed
        return encoder.getPosition() <= 0 && leftLimit;
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected boolean atForwardLimit() {
        // This is a safety. If the robot starts with the turret pointed in the
        // wrong direction, this will prevent us from ripping out the chain.
        if (!hasBeenZeroed) {
            return io_.velocity >= 0.0 && rightLimit;
        }

        // If were are here, the robot has been zeroed
        return encoder.getPosition() >= 0 && rightLimit;
    }

    /**
     * Get the rotation of the turret.
     * @return Gets the rotation of the turret in turret coordinates
     *         (i.e. 0 is towards the back of the robot.)
     */
    public synchronized Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getPosition());
    }

    /**
     * Get the rotation of the turret relative to the robot
     * @return Gets the rotation of the turret in robot coordinates
     *         (i.e. 0 is towards the front of the robot.)
     */
    public synchronized Rotation2d getTurretToRobot() {
        return getRotation().rotateBy(TURRET_TO_ROBOT);
    }

    @Override
    public boolean runActiveTests() {
        
        return false;
    }

    @Override
    public void zeroSensors() {
        encoder.setPosition(0.0);
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {

    }

    @Override
    public void outputTelemetry() {

    }
}
