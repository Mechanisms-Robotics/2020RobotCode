package frc2020.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.*;

import frc2020.loops.ILooper;
import frc2020.robot.Constants;
import frc2020.util.vision.GoalTracker;
import frc2020.util.vision.TargetInfo;

import java.util.List;

public class TargetTracker implements Subsystem {

    private List<Translation2d> cameraToVisionTarget_ = new ArrayList<>();
    private GoalTracker goalTracker_ = new GoalTracker();

    private Translation2d getCameraToVisionTargetPose(TargetInfo target, Limelight source) {
        
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ())
            .rotateBy(source.getHorizontalPlaneToLens());
        double x = xz_plane_translation.getX();
        double y = target.getY();
        double z = xz_plane_translation.getY();

        // Find the intersection with the goal
        double differential_height = source.getLensHeight() - Constants.TARGET_HEIGHT;
        if ((z < 0.0) == (differential_height > 0.0)) {
			double scaling = differential_height / -z;
			double distance = Math.hypot(x, y) * scaling;
			Rotation2d angle = new Rotation2d(x, y);
			return new Translation2d(distance * angle.getCos(), distance * angle.getSin());
		}
		return null;
    }

    private void updatePowerPortGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses, GoalTracker tracker) {
    	if (cameraToVisionTargetPoses.size() != 2 ||
				cameraToVisionTargetPoses.get(0) == null ||
						cameraToVisionTargetPoses.get(1) == null) return;

	}
	@Override
	public void writePeriodicOutputs() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void readPeriodicInputs() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean checkSystem() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void zeroSensors() {
		goalTracker_.reset();
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void registerLoops(ILooper enabledLooper) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void outputTelemetry() {
		// TODO Auto-generated method stub
		
	}

}