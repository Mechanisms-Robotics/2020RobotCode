package frc2020.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;
import java.util.Arrays;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc2020.subsystems.Limelight.LimelightRawData;
import frc2020.util.Logger;
import frc2020.util.Util;


/**
 * The target tracker is built to track Infinite Recharge power port vision targets.
 * The inputs to the class is raw data from the Limelight.  The class
 * maintains a list of readings along with confidences in those readings.  Each
 * time a reading is received, this class determines its confidence in the reading
 * based on various factors.  The reading is then added to a list of readings.
 *
 * Every time this class receives a reading, each reading in the list of readings
 * has its confidence decreased by a certain amount to account for decaying confidence
 * over time.  When a reading's confidence drops below a threshold, the reading is
 * discarded.  This method of depreciation of readings assumes that readings are
 * received at about the same time interval.
 *
 * The class uses a confidence-weighted average of all readings in the list to
 * determine an output reading.  This allows for smooth target tracking, confidence
 * determination, and dealing with spurious targets or rough target environments.
 */
public class TargetTracker {
    private final static double MIN_INITIAL_CONFIDENCE = 0.3;
    private final static double MIN_CONFIDENCE = 0.3;
    private final static double DEPRECIATION_FACTOR = 0.9;

    // Since readings will be called every 10 ms or so, we should never have more
    // than a second or two of readings.
    private final static int MAX_READINGS_SIZE = 150;

    private final static double TARGET_HEIGHT = 2.496; // meters TODO: join this with CAMERA_DH to avoid duplication?
    private static final double LIMELIGHT_RES_X = 320.0;
    private static final double LIMELIGHT_RES_Y = 240.0;

    private static final double LIMELIGHT_HORIZONTAL_FOV = 59.6;
    public static final double LIMELIGHT_VERTICAL_FOV = 49.7;

    // Defines the plane 1.0 unit away from the camera
    private static final double VERTICAL_PLANE_HEIGHT = 2.0 *
            Math.tan(Math.toRadians(LIMELIGHT_VERTICAL_FOV / 2.0));
    private static final double VERTICAL_PLANE_WIDTH = 2.0 *
            Math.tan(Math.toRadians(LIMELIGHT_HORIZONTAL_FOV / 2.0));

    private static Logger logger_ = Logger.getInstance();
    private final static String logName = "Limelight";

    /**
     * This internal class contains all of the most current readings from the limelight
     */
    public static class Reading {
        public double azimuth; // degrees (0 is ahead, positive to right)
        public double elevation; // degrees (0 is ahead, positive is up)
        public double range; // meters
        public double rangeArea;
        public double rangeCorner;
        public double confidence; // 0.0 to 1.0

        public Reading(double azimuth, double elevation, double range, double rangeArea, double rangeCorner, double confidence) {
            this.azimuth = azimuth;
            this.elevation = elevation;
            this.range = range;
            this.rangeArea = rangeArea;
            this.rangeCorner = rangeCorner;
            this.confidence = confidence;
        }
    }

    /**
     * Useful class for getting range with error calculated into it
     */
    public static class RangeAndError {
        public double error; //error is standard deviation
        public double range; 
    }

    // the list of readings
    public List<Reading> readings_ = new ArrayList<Reading>();
    private Limelight limelight_;

    public TargetTracker(Limelight limelight) {
        limelight_ = limelight;
    }

    /**
     * Call this function when there is new data to be added into the list of
     * readings.  This function should be called periodically (equal time spacing
     * between calls).
     */
    public void addLatestTargetData() {

        // Perform the initial confidence depreciation

        depreciateConfidences();

        // Pull all necessary raw data from the Limelight

        LimelightRawData rawData = limelight_.getRawData();

        // Perform an initial confidence pass on the raw data

        double initialConfidence = determineConfidenceInRawData(rawData);

        if (initialConfidence < MIN_INITIAL_CONFIDENCE) {
            return; // no reason to add this reading or to continue
        }

        // If the raw data looks okay, calculate the range and bearing

        Reading reading = determineAzimuthElevationAndRange(rawData);

        // Now determine the final confidence

        /* TODO: determine final confidence based on initial confidence and anything
         * else that makes sense.  Remember to make sure to constrain to 0 to 1.
         */

        reading.confidence = initialConfidence;  // TODO

        // Add the reading to the array, if it's good

        if (reading.confidence < MIN_CONFIDENCE) {
            return; // no reason to add this reading
        }

        if (readings_.size() <= MAX_READINGS_SIZE) {
            readings_.add(reading);
        } else {
            logger_.logWarning("Readings array reached maximum size!", logName);
        }
    }

    /**
     * Performs the confidence-weighted averaging and returns that reading.
     *
     * @return A new reading that is the current tracked position of the target.
     */
    public Reading getCurrentReading() {

        if (readings_.size() == 0) {
            // return a zero-confidence reading since we have no data
            return new Reading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        Reading averageReading = new Reading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        double confidenceSum = 0.0;

        // Perform weighted average readings
        for (int i = 0; i < readings_.size(); i++) {
            Reading reading = readings_.get(i);
            averageReading.azimuth += reading.confidence*readings_.get(i).azimuth;
            averageReading.elevation += reading.confidence*readings_.get(i).elevation;
            averageReading.range += reading.confidence*readings_.get(i).range;
            averageReading.rangeArea += reading.confidence*readings_.get(i).rangeArea;
            averageReading.rangeCorner += reading.confidence*readings_.get(i).rangeCorner;
            averageReading.confidence += reading.confidence*readings_.get(i).confidence;
            confidenceSum += reading.confidence;
        }

        averageReading.azimuth /= confidenceSum;
        averageReading.elevation /= confidenceSum;
        averageReading.range /= confidenceSum;
        averageReading.rangeArea /= confidenceSum;
        averageReading.rangeCorner /= confidenceSum;
        averageReading.confidence /= confidenceSum;

        return averageReading;
    }

    /**
     * Depreciates the confidence in all readings by a fixed amount, throwing
     * out anything that falls below the minimum confidence interval.
     */
    private void depreciateConfidences() {
        // Loop through every reading
        for (int i = 0; i < readings_.size(); i++) {

            // Depreciate the reading by the Depreciation Factor
            readings_.get(i).confidence *= DEPRECIATION_FACTOR;

            // Remove the reading if the confidence is too low
            if (readings_.get(i).confidence < MIN_CONFIDENCE) {
                readings_.remove(i);
                i--;
            }
        }
    }

    /**
     * Given raw data from the Limelight, return an initial confidence in the raw data
     *
     * @param rawData Raw data from the Limelight, taken all at once
     * @return A confidence in the interval of 0.0 to 1.0, inclusive
     */
    private double determineConfidenceInRawData(LimelightRawData rawData) {
        if (!rawData.hasTarget) {
            return 0.0;
        }

		// double[][] topCorners = {{rawData.corners[0], rawData.corners[1]}, {rawData.corners[2], rawData.corners[3]}};
		// double[][] bottomCorners = {{rawData.corners[4], rawData.corners[5]}, {rawData.corners[6], rawData.corners[7]}};
 
		// // Between 0.0-1.0
		// double score = 0.0;
 
		// double scoreInc = 0.125;
 
		// // This checks that the bottomCorners are lower than the topCorners by a reasonable amount
		// for (int j = 0; j < 2; j++) {
		// 	 for (int k = 0; k < 2; k++) {
		// 		 score += (Util.epsilonEquals(Math.abs(topCorners[j][1]-bottomCorners[k][1]), 
        //                    Constants.TOP_GOAL_DY, Constants.TARGETING_EPSILON)) ? scoreInc : 0.0;
        //      }
        //  }
         
		//  // This checks that the bottomCorners are inset on the x axis from the topCorners
		//  for (int j = 0; j < 2; j++) {
		// 	 for (int k = 0; k < 2; k++) {
		// 		 score += j%2==0?((topCorners[j][0]-bottomCorners[k][0])<0.0?scoreInc:0.0):
        //                          ((topCorners[j][0]-bottomCorners[k][0])>0.0?scoreInc:0.0);
        //      }
        //  }
		 
		//  // Checks to make sure the topCorners aren't too close together
		//  score += (Math.abs(topCorners[1][0]-topCorners[0][0]) >= Constants.TOP_GOAL_DX) ? scoreInc : 0.0;
 
		//  // Checks to make sure the bottomCorners aren't too close together
		//  score += (Math.abs(bottomCorners[1][0]-bottomCorners[0][0]) >= Constants.TOP_GOAL_BDX) ? scoreInc : 0.0;

        return 1.0;
    }

    /**
     * Given raw data from the Limelight, calculate the range and bearing.
     *
     * @param rawData Limelight raw readings
     * @return The calculated reading
     */
    private Reading determineAzimuthElevationAndRange(LimelightRawData rawData)
    {
        List<Translation2d> corners = getTopCorners(rawData);
        if (corners == null) {
            return new Reading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        Rotation2d[] cornerBasedTarget = getCornerBasedTarget(corners);

        RangeAndError rangeArea = getRangeFromArea(rawData);
        RangeAndError rangeCorner = getRangeUsingCorners(cornerBasedTarget);
        RangeAndError range = getRange(rangeArea, rangeCorner);

        // TODO: We should use error to return a confidence (in range 0.0 to 1.0)
        
        return new Reading(cornerBasedTarget[0].getDegrees(),
                           cornerBasedTarget[1].getDegrees(),
                           range.range,
                           rangeArea.range,
                           rangeCorner.range,
                           0.0);
    }

    /**
     * This function takes two different types of measurements for range and uses statistical
     * magic to combine them. See the 4910 programmer GroupMe from 1 Feb. 2020 with Carson's
     * formulas for combining the measurements statistically.
     * 
     * @param rangeArea
     * @param rangeCorner
     * @return
     */
    private RangeAndError getRange(RangeAndError rangeArea, RangeAndError rangeCorner) {
        RangeAndError rangeAndError = new RangeAndError();

        if (rangeArea.error == 0 && rangeCorner.error == 0) {
            rangeAndError.error = Math.abs(rangeArea.range - rangeCorner.range);
            rangeAndError.range = (rangeArea.range + rangeCorner.range)/2;
            return rangeAndError;
        }

        rangeAndError.range = (rangeArea.range * Math.pow(rangeCorner.error,2)) + (rangeCorner.range * Math.pow(rangeArea.error,2));
        rangeAndError.range /= (Math.pow(rangeArea.error,2) + Math.pow(rangeCorner.error,2));

        rangeAndError.error = rangeArea.error*rangeCorner.error*Math.sqrt(2);
        rangeAndError.error /= rangeArea.error*rangeArea.error + rangeCorner.error*rangeCorner.error;

        return rangeAndError;
    }

    /**
     * @param target The azimuth and elevation of the target
     * 
     * @return Range in meters to target
     */
    private RangeAndError getRangeUsingCorners(Rotation2d[] target) {
        Rotation2d angle = target[1];
        angle = angle.rotateBy(limelight_.getHorizontalPlaneToLens());
        
        double differental_height = TARGET_HEIGHT - limelight_.getLensHeight();
        double angleTan = angle.getTan();

        if(angleTan == 0) {
            RangeAndError rangeAndError = new RangeAndError();
            rangeAndError.error = 10.0; // 10 meters
            rangeAndError.range = 10.0; // 10 meters
            return rangeAndError;
        }

        RangeAndError rangeAndError = new RangeAndError();
        rangeAndError.error = 0.5; //TODO: replace with more accurate statistics
        rangeAndError.range = differental_height / angleTan;
        
        return rangeAndError;
    }

    /** Uses two interpolated functions to calculate range based on the width and height of the target.
     *  Width and height ratio used to calculate constants of proportionality for area function.
     *  Area function calculated with constants using the pixel area of the target.
     * 
     * @param rawData Limelight raw readings
     * @return range in meters
     */
    private RangeAndError getRangeFromArea(LimelightRawData rawData) {
        double range = 0;
        double am = 3.20505; //slope of "a" function
        double ab = 168.682; //constant of "a" function
        double bm = -0.395496; //slope for "b" function
        double bb = 0.317853; //constant for "b" function
        double area = rawData.tWidth * rawData.tHeight;
        if(rawData.tHeight == 0 || rawData.tWidth == 0){
            RangeAndError rangeAndError = new RangeAndError();
            rangeAndError.error = 10.0; // 10 meters
            rangeAndError.range = 10.0; // 10 meters
            return rangeAndError;
        }
        double whRatio = rawData.tWidth/rawData.tHeight;

        double a = am*whRatio + ab;
        double b = bm*whRatio + bb;
        range = a / Math.sqrt(area) + b;

        RangeAndError rangeAndError = new RangeAndError();
        rangeAndError.error = 0.5; //TODO: replace later with better statistics
        rangeAndError.range = range;

        return rangeAndError;
    }

    /**
     * @param data Raw limelight data
     * 
     * @return The top two corners as Translation2d's
     */
    private static List<Translation2d> getTopCorners(Limelight.LimelightRawData data) {
        // If we don't have a valid target or the corners list is empty
        // something went wrong so we return null
        double[] emptyArray = {};
        if (!data.hasTarget  || Arrays.equals(data.corners, emptyArray)  || data.corners.length <= 8){
            return null;
        }
        
        List<Translation2d> corners = new ArrayList<Translation2d>();

        for (int i = 0; i < data.corners.length-2; i+=2) {
            corners.add(new Translation2d(data.corners[i], data.corners[i+1]));
        }

        Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::getY);
        corners.sort(ySort);
        
        return corners;   
    }
    
    /**
     * Converts pixels to angles using math on limelight page
     */
    private static Rotation2d[] pixelToAngle(Translation2d pixelCoord) {
        // Convert to angle
        double x_pixels = pixelCoord.getX();
        double y_pixels = pixelCoord.getY();

        double nx = -((x_pixels - (LIMELIGHT_RES_X / 2.0)) /
            (LIMELIGHT_RES_X / 2.0));
        double ny = (y_pixels - (LIMELIGHT_RES_Y / 2.0)) /
            (LIMELIGHT_RES_Y / 2.0);
        

        double x = VERTICAL_PLANE_WIDTH / 2 * nx;
        double y = VERTICAL_PLANE_HEIGHT / 2 * ny;

        return new Rotation2d[] {new Rotation2d(Math.atan2(1.0, x) - (Math.PI/2.0)),
            new Rotation2d(Math.atan2(1.0, y) - (Math.PI/2.0))};
    }
    
    /**
     * Raw target info of top two corners of target in unit plane of -1 to 1
     * 
     * @param topCorners two top corners recieved from limelight
     */
    private static Rotation2d[] getCornerBasedTarget(List<Translation2d> topCorners) {
        Translation2d centerPoint = Util.interpolateTranslation2d(topCorners.get(0), topCorners.get(1), 0.5);
        return pixelToAngle(centerPoint);
    }
}
