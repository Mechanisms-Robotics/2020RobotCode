package frc2020.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class InterpolatingPose2d extends Pose2d implements Interpolable<InterpolatingPose2d> {

    public InterpolatingPose2d(Translation2d translation2d, Rotation2d rotation2d) {
        super(translation2d, rotation2d);
    }

    public InterpolatingPose2d(Pose2d pose) {
        super(pose.getTranslation(), pose.getRotation());
    }

    @Override
    public InterpolatingPose2d interpolate(InterpolatingPose2d other, double x) {
        return new InterpolatingPose2d(Util.interpolatePose2d(this, other, x));
    }
}
