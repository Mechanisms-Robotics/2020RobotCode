package frc2020.util.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

import java.util.LinkedList;

/**
 * Driver for an analog Ultrasonic Sensor (mainly to help smooth out noise).
 */
public class UltrasonicSensor {
    protected AnalogInput mAnalogInput;
    private LinkedList<Double> cache;
    protected double mScalingFactor = 512.0 / 5.0;

    private static final int kCacheSize = 5;

    /**
     * Constructs new ultrasonic sensor
     * 
     * @param port port of sensor
     */
    public UltrasonicSensor(int port) {
        mAnalogInput = new AnalogInput(port);
        cache = new LinkedList<Double>();
        cache.add(getRawDistance());
    }

    /**
     * Update the reading. Should be called periodically.
     */
    public void update() {
        cache.add(getRawDistance());
        if (cache.size() > kCacheSize)
            cache.removeFirst();
    }

    /**
     * @return The distance in cm
     */
    public double getRawDistance() {
        return mAnalogInput.getVoltage() * mScalingFactor;
    }

    /**
     * @return The average Distance
     */
    public double getAverageDistance() {
        double total = 0;
        for (Double d : cache) {
            total += d;
        }
        return total / cache.size();
    }

    /**
     * @return The last distance read
     */
    public double getLatestDistance() {
        return cache.getLast();
    }
}
