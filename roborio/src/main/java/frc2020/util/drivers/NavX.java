package frc2020.util.drivers;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;


/**
 * Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
 */
public class NavX {
    protected class Callback implements ITimestampedDataSubscriber {

        @Override
        public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase update,
                                            Object context) {
            synchronized (NavX.this) {
                // This handles the fact that the sensor is inverted from our coordinate conventions.
                if (mLastSensorTimestampMs != kInvalidTimestamp && mLastSensorTimestampMs < sensor_timestamp) {
                    mYawRateDegreesPerSecond = 1000.0 * (-mYawDegrees - update.yaw)
                            / (double) (sensor_timestamp - mLastSensorTimestampMs);
                }
                mLastSensorTimestampMs = sensor_timestamp;
                mYawDegrees = -update.yaw;
            }
        }
    }

    protected AHRS mAHRS;

    protected Rotation2d mAngleAdjustment = new Rotation2d();
    protected double mYawDegrees;
    protected double mYawRateDegreesPerSecond;
    protected final long kInvalidTimestamp = -1;
    protected long mLastSensorTimestampMs;

    /**
     * Constructs new navx using SPI ports protocol
     * 
     * @param spi_port_id id of navx
     */
    public NavX(SPI.Port spi_port_id) {
        mAHRS = new AHRS(spi_port_id, (byte) 200);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    /**
     * Constructs new navx
     * 
     * @param serial_port_id the id of the navx
     */
    public NavX(SerialPort.Port serial_port_id) {
        mAHRS = new AHRS(serial_port_id);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    /**
     * Reset the NavX
     */
    public synchronized void reset() {
        mAHRS.reset();
        resetState();
    }

    /**
     * Set the yaw to 0 and reset the NavX state
     */
    public synchronized void zeroYaw() {
        mAHRS.zeroYaw();
        resetState();
    }

    /**
     * Resets the NavX
     */
    private void resetState() {
        mLastSensorTimestampMs = kInvalidTimestamp;
        mYawDegrees = 0.0;
        mYawRateDegreesPerSecond = 0.0;
    }

    /**
     * Set the angle to transform all reading by. Generally used for when the navx is mounted at an angle or
     * or the robot is turned on crooked.
     *
     * @param adjustment
     */
    public synchronized void setAngleAdjustment(Rotation2d adjustment) {
        mAngleAdjustment = adjustment;
    }

    /**
     * @return The raw yaw reading of the navx in degrees
     */
    protected synchronized double getRawYawDegrees() {
        return mYawDegrees;
    }

    /**
     * @return The unadjusted yaw of navx as a Rotation2d
     */
    public synchronized Rotation2d getRawRotation() {
        return Rotation2d.fromDegrees(getRawYawDegrees());
    }

    /**
     * @return The true yaw of the navx as a rotation 2d
     */
    public Rotation2d getYaw() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
    }

    /**
     * @return How fast the yaw is changing in degrees per sec
     */
    public double getYawRateDegreesPerSec() {
        return mYawRateDegreesPerSecond;
    }

    /**
     * @return How fast the yaw is changing in radians per sec
     */
    public double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }

    /**
     * @return The navx acceleration reading on the x axis.
     */
    public double getRawAccelX() {
        return mAHRS.getRawAccelX();
    }

    /**
     * @return The navx roll in degrees
     */
    public double getRollDegrees() {
        return mAHRS.getRoll();
    }

    /**
     * @return The navx pitch in degrees
     */
    public double getPitchDegrees() {
        return mAHRS.getPitch();
    }

    public long getLastSensorTimestamp() {
        return mLastSensorTimestampMs;
    }

    public boolean isConnected() {
        return mAHRS.isConnected();
    }

}
