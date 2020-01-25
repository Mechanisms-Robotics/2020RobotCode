package frc2020.auto.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc2020.subsystems.Drive;
import frc2020.util.Logger;

public class DriveTrajectory implements Command {
    public static final Drive mDrive = Drive.getInstance();

    private final Trajectory mTrajectory;
    private final boolean mResetPose;

    public DriveTrajectory(Trajectory trajectory) {
        this(trajectory, false);
    }

    public DriveTrajectory(Trajectory trajectory, boolean resetPose) {
        mTrajectory = trajectory;
        mResetPose = resetPose;

    }
    
    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()){
            Logger.getInstance().logInfo("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        Logger.getInstance().logInfo("Starting Trajectory!");
        if (mResetPose){
            Pose2d startPose = mTrajectory.getInitialPose();
            mDrive.resetOdometry(startPose, startPose.getRotation());
        }
        mDrive.driveTrajectory(mTrajectory);

    }

}