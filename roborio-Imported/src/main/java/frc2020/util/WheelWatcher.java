
package frc2020.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc2020.util.TCS34725_I2C.TransferAbortedException; // ONLY FOR FIVES
// import frc2020.util.TCS34725_I2C.TCSColor; // ONLY FOR FIVES
// import frc.robot.TCS34725_I2C.TransferAbortedException;
public class WheelWatcher {

    /**
     * UNCOMMENT THIS FOR SIXES
     */

    // public enum WheelColor {
    //     UNKNOWN, RED, YELLOW, BLUE, GREEN
    // };
    
    // // For now, at least, we only support this device.
    // // TODO: abstract color acquisition into an interface.
    // private final TCS34725_I2C m_colorSensor = new TCS34725_I2C();
    // private boolean m_sensorOK = false;

    // // Making sure we get consistent color readings.
    // private WheelColor m_sameColorSeen = WheelColor.UNKNOWN;
    // private Timer m_sameColorTimer = new Timer();
    // private long m_unknownCount;

    // // Tuning parameters.
    // private double m_sameColorTimeThreshold = 0.10;
    // private double m_colorDiffFactor = 1.2;

    // // Managing color state.
    // private WheelColor m_readColor = WheelColor.UNKNOWN;
    // private WheelColor m_lastWedgeColor = WheelColor.UNKNOWN;
    // private Timer m_inWedgeTimer = new Timer();
    // private Timer m_rotationTimer = new Timer();
    // private long m_changeCount;

    // // Tracking wheel movement.
    // private double m_wedgeRPM;
    // private double m_averageRPM;

    // public WheelWatcher() {
    //     init();
    // }

    // /**
    //  * Convert a sensor reading into one of our wheel colors.
    //  * 
    //  * @param tColor raw sensor reading
    //  * @return current WheelColor value
    //  */
    // protected WheelColor getWheelColor(TCSColor tColor) {
    //     double r = tColor.getR();
    //     double g = tColor.getG();
    //     double b = tColor.getB();

    //     double rt = r * m_colorDiffFactor;
    //     double gt = g * m_colorDiffFactor;
    //     double bt = b * m_colorDiffFactor;

    //     if (r > gt && r > bt) {
    //         return WheelColor.RED;
    //     }

    //     if (g > rt && g > bt) {
    //         return WheelColor.GREEN;
    //     }

    //     if (b > rt && b > gt) {
    //         return WheelColor.BLUE;
    //     }

    //     if (r > bt && g > bt) {
    //         return WheelColor.YELLOW;
    //     }

    //     // We also get dominant blue AND green when
    //     // we're on the blue segment.
    //     if (b > rt && g > rt) {
    //         return WheelColor.BLUE;
    //     }

    //     return WheelColor.UNKNOWN;

    // }

    // public void init() {
    //     try {
    //         m_colorSensor.enable();
    //         m_colorSensor.setIntegrationTime(TCS34725_I2C.TCS34725_INTEGRATIONTIME_2_4MS);
    //         m_sensorOK = true;
    //     } catch (Exception ex) {
    //         m_sensorOK = false;
    //         ex.printStackTrace();
    //     }
    //     reset();
    // }

    // public void reset() {
    //     System.out.println("Resetting WheelWatcher");
    //     m_sameColorSeen = WheelColor.UNKNOWN;
    //     m_lastWedgeColor = WheelColor.UNKNOWN;
    //     m_sameColorTimer.start();
    //     m_inWedgeTimer.start();
    //     m_unknownCount = 0;
    //     m_changeCount = 0;
    //     m_wedgeRPM = 0.0;
    //     m_averageRPM = 0.0;
    // }

    // public void update() {
    //     try {
    //         TCS34725_I2C.TCSColor color = m_colorSensor.getRawData();
    //         SmartDashboard.putNumber("Red", color.getR());
    //         SmartDashboard.putNumber("Green", color.getG());
    //         SmartDashboard.putNumber("Blue", color.getB());
    //         m_readColor = getWheelColor(color);
    //         m_sensorOK = true;

    //     } catch (TransferAbortedException ex) {
    //         SmartDashboard.putString("Sensor Status", "FAILED " + ex.toString());
    //         m_sensorOK = false;
    //         return;
    //     }

    //     // If we don't recognize the color, note an unknown, and bail.
    //     if (m_readColor == WheelColor.UNKNOWN) {
    //         m_unknownCount++;
    //         return;
    //     }

    //     // We don't want to do anything about the color until
    //     // we've gotten that reading consistently for at least
    //     // a while.
    //     if (m_readColor != m_sameColorSeen) {
    //         m_sameColorSeen = m_readColor;
    //         m_sameColorTimer.reset();
    //         return;
    //     }

    //     // If we haven't seen the same color consistently for
    //     // a brief amount of time yet, nothing to do yet.
    //     if (m_sameColorTimer.get() < m_sameColorTimeThreshold) {
    //         return;
    //     }

    //     // Okay, we're confident we're in a color. If it's
    //     // the same color we were in, nothing to do yet; we're
    //     // just watching it go by.
    //     if (m_readColor == m_lastWedgeColor) {
    //         return;
    //     }

    //     // Now we're confidently in a color that is not the same color we WERE in. But
    //     // we need that to happen TWICE before we can start checking position. At the
    //     // first transition we don't know where we started relative to the radius; just
    //     // update where we are now.
    //     m_changeCount++;
    //     if (m_changeCount < 2) {
    //         m_lastWedgeColor = m_readColor;
    //         return;
    //     }

    //     // If we just hit our second edge, start the long-term rotation timer.
    //     if (m_changeCount == 2) {
    //         m_rotationTimer.reset();
    //         m_rotationTimer.start();
    //     }

    //     // Finally, the interesting case; we knew we were in one color
    //     // (m_lastWedgeColor)
    //     // but now we're in a different one (readColor) and we've seen at least TWO
    //     // edge transitions.

    //     // LATER: confirm expected transition, and wbat to do if it fails? just reset
    //     // our state?

    //     // Now remember our last wedge color, get the current elapsed time, and reset
    //     // the timer.
    //     m_lastWedgeColor = m_readColor;
    //     double inWedgeElapsed = m_inWedgeTimer.get();
    //     m_inWedgeTimer.reset();

    //     // a wedge is 45 degrees, so rotational velocity in degrees per second
    //     double degreesPerSecond = 45.0 / inWedgeElapsed;

    //     // a revolution is 360 degrees, and a minute is 60 seconds, so...
    //     m_wedgeRPM = (degreesPerSecond * 60.0) / 360.0;

    //     // average RPM is 60 * ((change count - 2) / 8) / rotation elapsed time.
    //     m_averageRPM = 60.0 * ((m_changeCount - 2) / 8.0) / m_rotationTimer.get();

    // }

    // // Getters.
    // public long getUnknownCount() {
    //     return m_unknownCount;
    // }

    // public double getWedgeRPM() {
    //     return m_wedgeRPM;
    // }

    // public double getAverageRPM() {
    //     return m_averageRPM;
    // }

    // public long getEdgeCount() {
    //     return m_changeCount - 1;
    // }

    // public boolean isSensorOK() {
    //     return m_sensorOK;
    // }

    // public WheelColor getWedgeColor() {
    //     return m_lastWedgeColor;
    // }

    // // Setters for tuning parameters.

    // /**
    //  * Set the amount of time we need to see consistent color values
    //  * before we decide we're actually in that color.
    //  * 
    //  * @param val time in seconds
    //  */
    // public void setSameColorTimeThreshold(double val) {
    //     m_sameColorTimeThreshold = val;
    // }

    // /**
    //  * Set the factor by which the dominant color(s) need to
    //  * exceed the other color(s) to conclude a wheel color.
    //  * 
    //  * @param val dominance factor
    //  */
    // public void setColorDiffFactor(double val) {
    //     m_colorDiffFactor = val;
    // }

    
    // /**
    //  * Return the color at 90 degrees 
    //  * @param c the base color
    //  * @return the color at 90 degrees from the base color
    //  */
    // public WheelColor getColorAt90(WheelColor c) {
    //     switch (c) {
    //         case RED:
    //             return WheelColor.BLUE;
    //         case YELLOW:
    //             return WheelColor.GREEN;
    //         case BLUE:
    //             return WheelColor.RED;
    //         case GREEN:
    //             return WheelColor.YELLOW;
    //         default:
    //             return WheelColor.UNKNOWN;
    //     }
    // }

}