package frc2020.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.loops.Loop;
import frc2020.robot.Constants;
import frc2020.util.Util;
import frc2020.util.vision.GoalTracker;
import frc2020.util.vision.TargetInfo;

import java.util.List;

public class TargetTracker implements Subsystem {

    private List<Translation2d> cameraToVisionTarget_ = new ArrayList<>();
    private GoalTracker goalTracker_ = new GoalTracker();

   	// TODO: Replace suse of this with getting this from the turret when the turret is done
	private static final Pose2d TRANSFORM_TO_TURRET = new Pose2d(0, 0, Rotation2d.fromDegrees(180.0));

	private Limelight limelight_;
	public TargetTracker(Limelight limelight) {
		limelight_ = limelight;
	}

	/**
	 * Return the distance to a target seen by a limelight
	 * @param target The target to calculate the distance too
	 * @param source The limelight that was used to find the target
	 * @return a translation 2d that represents the placement of the target relative to the limelight
	 * @see TargetInfo
	 * @see Limelight
	 * @see Translation2d
	 */
    private static Translation2d getCameraToVisionTargetPose(TargetInfo target, Limelight source) {
        
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ())
            .rotateBy(source.getHorizontalPlaneToLens());
        double x = xz_plane_translation.getX();
        double y = target.getY();
        double z = xz_plane_translation.getY();
        Rotation2d angle = new Rotation2d(target.getX(), target.getZ());

        // Find the intersection with the goal
        double differential_height =  Constants.TARGET_HEIGHT - source.getLensHeight();
        double test_distance = differential_height/(Math.tan(angle.getRadians() + Math.toRadians(15.0)));
		SmartDashboard.putNumber("Target Distance", test_distance);
        if (z > 0.0) {
			double scaling = differential_height / z;
			double distance = Math.hypot(x, y) * scaling;
			//Rotation2d angle = new Rotation2d(x, y);
			return new Translation2d(distance * angle.getCos(), distance * angle.getSin());
		}
		return null;
    }

    private static void updateGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses,
										  GoalTracker tracker,
										  Limelight source) {
    	if (cameraToVisionTargetPoses.size() != 2 ||
				cameraToVisionTargetPoses.get(0) == null ||
						cameraToVisionTargetPoses.get(1) == null) return;

    	// Get the center of the target from the two corners
		Translation2d target_center = Util.interpolateTranslation2d(cameraToVisionTargetPoses.get(0),
				cameraToVisionTargetPoses.get(1), 0.5);
		Transform2d transform = new Transform2d(
				target_center,
				new Rotation2d()
		);
		Pose2d fieldToVisionTarget = TRANSFORM_TO_TURRET.transformBy(source.getTurretToLens()).
				transformBy(transform);
		tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), new Rotation2d())));
	}

	private final Loop trackingLoop = new Loop() {

    	public void init() {
    		synchronized (this) {
    			goalTracker_.reset();
    			cameraToVisionTarget_.clear();
			}
		}

		public void run() {
    		synchronized (this) {
				double timestamp = Timer.getFPGATimestamp() - limelight_.getLatency();
				List<TargetInfo> targets = limelight_.getTarget();

				cameraToVisionTarget_.clear();
				if (targets == null || targets.isEmpty()) {
					goalTracker_.update(timestamp, new ArrayList<>());
					return;
				}

				for (TargetInfo target : targets) {
					cameraToVisionTarget_.add(getCameraToVisionTargetPose(target, limelight_));
				}

				updateGoalTracker(timestamp, cameraToVisionTarget_, goalTracker_, limelight_);
			}
		}

		public void end() {
			goalTracker_.reset();
			cameraToVisionTarget_.clear();
		}
	};

	@Override
	public void writePeriodicOutputs() {
		// Nothing to do here
	}

	public synchronized Pose2d getFieldToVisionTarget() {
		if (!goalTracker_.hasTracks()) {
			return null;
		}
		Pose2d fieldToTarget = goalTracker_.getTracks().get(0).field_to_target;
		return new Pose2d(fieldToTarget.getTranslation(),
				Rotation2d.fromDegrees(Constants.TARGET_NORMAL));
	}

	public synchronized Pose2d getRobotToVisionTarget() {
		Pose2d fieldToVisionTarget = getFieldToVisionTarget();
		if (fieldToVisionTarget == null) {
			return null;
		}
		Transform2d transform = Util.poseToTransform(fieldToVisionTarget);
		Pose2d robotToField = Drive.getInstance().getOdometryPose(Timer.getFPGATimestamp() - limelight_.getLatency());
		return Util.invertPose2d(robotToField).transformBy(transform);
	}

	@Override
	public void readPeriodicInputs() {

	}

	@Override
	public boolean checkSystem() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void zeroSensors() {
		goalTracker_.reset();
		cameraToVisionTarget_.clear();
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void registerLoops(ILooper enabledLooper) {
		enabledLooper.register(trackingLoop);
	}

	@Override
	public void outputTelemetry() {
		Pose2d target_pose = getRobotToVisionTarget();
		if (target_pose != null) {
			SmartDashboard.putNumber("Target X", target_pose.getTranslation().getX());
			SmartDashboard.putNumber("Target Y", target_pose.getTranslation().getY());
			SmartDashboard.putNumber("Target Rotation", target_pose.getRotation().getDegrees());
		} else {
			SmartDashboard.putNumber("Target X", 0.0);
			SmartDashboard.putNumber("Target Y", 0.0);
			SmartDashboard.putNumber("Target Rotation", 0.0);
		}
	}

}