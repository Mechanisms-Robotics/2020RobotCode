package frc2020.auto.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc2020.subsystems.Drive;
import frc2020.util.Logger;

/**
 * Constructs trajectory so it can now be run by runCommand
 * parameter Trajectory
 */
public class DriveTrajectory implements Command {
    public static final Drive mDrive = Drive.getInstance();

    private final Trajectory mTrajectory;
    private final boolean mResetPose;

    private static Logger logger_ = Logger.getInstance();

    /**
     * Constructs trajectory and doesn't reset pose (use anytime other than 
     * first trajectory)
     * @param trajectory
     */
    public DriveTrajectory(Trajectory trajectory) {
        this(trajectory, false);
    }

    /**
     * Constructs trajectory and resets pose
     * @param trajectory
     * @param resetPose set to true only if this is the very first trajectory
     */
    public DriveTrajectory(Trajectory trajectory, boolean resetPose) {
        mTrajectory = trajectory;
        mResetPose = resetPose;

    }
    
    /**
     * Logs "Trajectory Finished" if true
     */
    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()){
            logger_.logInfo("Trajectory finished");
            return true;
        }
        return false;
    }

    /**
     * 
     */
    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    /**
     * Logs "Starting Trajectory!" and if told to reset pose then
     * resets pose and odometry. At the end it sets the drive
     * instance to drive the current trajectory
     */
    @Override
    public void start() {
        logger_.logInfo("Starting Trajectory!");
        if (mResetPose){
            Pose2d startPose = mTrajectory.getInitialPose();
            mDrive.resetOdometry(startPose, startPose.getRotation());
        }
        mDrive.driveTrajectory(mTrajectory);

    }

}