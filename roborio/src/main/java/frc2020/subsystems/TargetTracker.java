package frc2020.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;
import java.util.Arrays;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.subsystems.Limelight.LimelightRawData;
import frc2020.robot.Constants;
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
    private final static int MAX_READINGS_SIZE = 200;

    private static Logger logger_ = Logger.getInstance();
    private final static String logName = "Limelight";

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

    public static class RangeAndError {
        public double error;//error is standard deviation
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

        // TODO: determine final confidence based on initial confidence and anything
        // else that makes sense.  Remember to make sure to constrain to 0 to 1

        reading.confidence = initialConfidence;  // TODO

        // Add the reading to the array, if it's good

        if (reading.confidence < MIN_CONFIDENCE) {
            return; // no reason to add this reading
        }

        if (readings_.size() <= MAX_READINGS_SIZE){
            readings_.add(reading);
        } else {
            logger_.logError("Readings array reached maximum size!", logName);
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
        for (int i = 0; i < readings_.size(); i++){
            averageReading.azimuth += readings_.get(i).confidence*readings_.get(i).azimuth;
            averageReading.elevation += readings_.get(i).confidence*readings_.get(i).elevation;
            averageReading.range += readings_.get(i).confidence*readings_.get(i).range;
            averageReading.rangeArea += readings_.get(i).confidence*readings_.get(i).rangeArea;
            averageReading.rangeCorner += readings_.get(i).confidence*readings_.get(i).rangeCorner;
            averageReading.confidence += readings_.get(i).confidence*readings_.get(i).confidence;
            confidenceSum += readings_.get(i).confidence;
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

            //Remove the reading if the confidence is too low
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
     * Given raw data from the Limelight, calculate the range and bearing.  The
     * confidence on the returned reading is zero, and should be reset before use.
     *
     * @param rawData Limelight raw readings
     * @return Returns a ZERO CONFIDENCE range and bearing
     */
    private Reading determineAzimuthElevationAndRange(LimelightRawData rawData)
    {
        // TODO: Modify this function to take whatever data we need to determine the
        // azimuth, elevation and range and calculate it.  Return a ZERO CONFIDENCE
        // reading.

        // Calcutae conner based target
        // getTopConers()
        // getConerBasdeTarget()

        List<Translation2d> corners = getTopCorners(rawData);
        if (corners == null) {
            return new Reading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        Rotation2d[] cornerBasedTarget = getCornerBasedTarget(corners);
        double confidence = 0.0;
        RangeAndError rangeArea = getRangeFromArea(rawData);
        RangeAndError rangeCorner = getRangeUsingCorners(cornerBasedTarget);
        
        return new Reading(cornerBasedTarget[0].getDegrees(),
                           cornerBasedTarget[1].getDegrees(),
                           getRange(rangeArea, rangeCorner).range,
                           rangeArea.range,
                           rangeCorner.range,
                           confidence);

        // TODO: we should use the error from getRange above in our eventual confidence
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
        
        double differental_height = Constants.TARGET_HEIGHT - limelight_.getLensHeight();
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
     * @param pixelCoord taken in pixels
     */
    private static Rotation2d[] pixelToAngle(Translation2d pixelCoord) {
        // Convert to angle
        double x_pixels = pixelCoord.getX();
        double y_pixels = pixelCoord.getY();

        double nx = -((x_pixels - (Constants.LIMELIGHT_RES_X / 2.0)) / 
            (Constants.LIMELIGHT_RES_X / 2.0));
        double ny = (y_pixels - (Constants.LIMELIGHT_RES_Y / 2.0)) / 
            (Constants.LIMELIGHT_RES_Y / 2.0);
        

        double x = Constants.VERTICAL_PLANE_WIDTH / 2 * nx;
        double y = Constants.VERTICAL_PLANE_HEIGHT / 2 * ny;

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

//import edu.wpi.first.wpilibj.Timer;
//        import edu.wpi.first.wpilibj.geometry.*;
//
//        import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//        import frc2020.loops.ILooper;
//        import frc2020.loops.Loop;
//        import frc2020.robot.Constants;
//        import frc2020.util.Util;
//        import frc2020.util.vision.GoalTracker;
//        import frc2020.util.vision.TargetInfo;


//public class TargetTracker implements Subsystem {
//
//    private List<Translation2d> cameraToVisionTarget_ = new ArrayList<>();
//    private GoalTracker goalTracker_ = new GoalTracker();
//
//   	// TODO: Replace suse of this with getting this from the turret when the turret is done
//	private static final Pose2d TRANSFORM_TO_TURRET = new Pose2d(0, 0, Rotation2d.fromDegrees(180.0));
//
//	private Limelight limelight_;
//	public TargetTracker(Limelight limelight) {
//		limelight_ = limelight;
//	}
//
//	/**
//	 * Return the distance to a target seen by a limelight
//	 * @param target The target to calculate the distance too
//	 * @param source The limelight that was used to find the target
//	 * @return a translation 2d that represents the placement of the target relative to the limelight
//	 * @see TargetInfo
//	 * @see Limelight
//	 * @see Translation2d
//	 */
//    private static Translation2d getCameraToVisionTargetPose(TargetInfo target, Limelight source) {
//
//        // Compensate for camera pitch
//        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ())
//            .rotateBy(source.getHorizontalPlaneToLens());
//        double x = xz_plane_translation.getX();
//        double y = target.getY();
//        double z = xz_plane_translation.getY();
//        Rotation2d angle = new Rotation2d(target.getX(), target.getZ());
//
//        // Find the intersection with the goal
//        double differential_height =  Constants.TARGET_HEIGHT - source.getLensHeight();
//        double test_distance = differential_height/(Math.tan(angle.getRadians() + Math.toRadians(15.0)));
//		SmartDashboard.putNumber("Target Distance", test_distance);
//        if (z > 0.0) {
//			double scaling = differential_height / z;
//			double distance = Math.hypot(x, y) * scaling;
//			//Rotation2d angle = new Rotation2d(x, y);
//			return new Translation2d(distance * angle.getCos(), distance * angle.getSin());
//		}
//		return null;
//    }
//
//    private static void updateGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses,
//										  GoalTracker tracker,
//										  Limelight source) {
//    	if (cameraToVisionTargetPoses.size() != 2 ||
//				cameraToVisionTargetPoses.get(0) == null ||
//						cameraToVisionTargetPoses.get(1) == null) return;
//
//    	// Get the center of the target from the two corners
//		Translation2d target_center = Util.interpolateTranslation2d(cameraToVisionTargetPoses.get(0),
//				cameraToVisionTargetPoses.get(1), 0.5);
//		Transform2d transform = new Transform2d(
//				target_center,
//				new Rotation2d()
//		);
//		Pose2d fieldToVisionTarget = TRANSFORM_TO_TURRET.transformBy(source.getTurretToLens()).
//				transformBy(transform);
//		tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), new Rotation2d())));
//	}
//
//	private final Loop trackingLoop = new Loop() {
//
//    	public void init() {
//    		synchronized (this) {
//    			goalTracker_.reset();
//    			cameraToVisionTarget_.clear();
//			}
//		}
//
//		public void run() {
//    		synchronized (this) {
//				double timestamp = Timer.getFPGATimestamp() - limelight_.getLatency();
//				List<TargetInfo> targets = limelight_.getTarget();
//
//				cameraToVisionTarget_.clear();
//				if (targets == null || targets.isEmpty()) {
//					goalTracker_.update(timestamp, new ArrayList<>());
//					return;
//				}
//
//				for (TargetInfo target : targets) {
//					cameraToVisionTarget_.add(getCameraToVisionTargetPose(target, limelight_));
//				}
//
//				updateGoalTracker(timestamp, cameraToVisionTarget_, goalTracker_, limelight_);
//			}
//		}
//
//		public void end() {
//			goalTracker_.reset();
//			cameraToVisionTarget_.clear();
//		}
//	};
//
//	@Override
//	public void writePeriodicOutputs() {
//		// Nothing to do here
//	}
//
//	public synchronized Pose2d getFieldToVisionTarget() {
//		if (!goalTracker_.hasTracks()) {
//			return null;
//		}
//		Pose2d fieldToTarget = goalTracker_.getTracks().get(0).field_to_target;
//		return new Pose2d(fieldToTarget.getTranslation(),
//				Rotation2d.fromDegrees(Constants.TARGET_NORMAL));
//	}
//
//	public synchronized Pose2d getRobotToVisionTarget() {
//		Pose2d fieldToVisionTarget = getFieldToVisionTarget();
//		if (fieldToVisionTarget == null) {
//			return null;
//		}
//		Transform2d transform = Util.poseToTransform(fieldToVisionTarget);
//		Pose2d robotToField = Drive.getInstance().getOdometryPose(Timer.getFPGATimestamp() - limelight_.getLatency());
//		return Util.invertPose2d(robotToField).transformBy(transform);
//	}
//
//	@Override
//	public void readPeriodicInputs() {
//
//	}
//
//	@Override
//	public boolean checkSystem() {
//		// TODO Auto-generated method stub
//		return false;
//	}
//
//	@Override
//	public void zeroSensors() {
//		goalTracker_.reset();
//		cameraToVisionTarget_.clear();
//	}
//
//	@Override
//	public void stop() {
//		// TODO Auto-generated method stub
//
//	}
//
//	@Override
//	public void registerLoops(ILooper enabledLooper) {
//		enabledLooper.register(trackingLoop);
//	}
//
//	@Override
//	public void outputTelemetry() {
//		Pose2d target_pose = getRobotToVisionTarget();
//		if (target_pose != null) {
//			SmartDashboard.putNumber("Target X", target_pose.getTranslation().getX());
//			SmartDashboard.putNumber("Target Y", target_pose.getTranslation().getY());
//			SmartDashboard.putNumber("Target Rotation", target_pose.getRotation().getDegrees());
//		} else {
//			SmartDashboard.putNumber("Target X", 0.0);
//			SmartDashboard.putNumber("Target Y", 0.0);
//			SmartDashboard.putNumber("Target Rotation", 0.0);
//		}
//	}
//
//}
