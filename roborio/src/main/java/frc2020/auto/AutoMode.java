package frc2020.auto;

import frc2020.auto.commands.Command;
import frc2020.util.Logger;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 *
 * @author Team 254
 * @author Modified by Team 4910
 */
public abstract class AutoMode {

    protected double update_rate = 1.0 / 50.0; // How fast the auto mode runs
    protected boolean running = false;
    protected boolean hasRun = false;

    protected abstract void routine() throws AutoModeEndedException;

    /**
    * Will run the AutoMode's routine until it is finished
    */
    public void run() {
        running = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            Logger.getInstance().logWarning("Auto ended and we weren't done!");
        }

        done();
        Logger.getInstance().logInfo("Auto mode done");
    }

    public void done() {
        hasRun = true;
    }

    public void stop() {
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    public boolean isRunningWithThrow() throws AutoModeEndedException {
        if (!isRunning()) {
            throw new AutoModeEndedException();
        }

        return isRunning();
    }

    /**
    * Will start command then update it every 20 milliseconds until finished
    */
    public void runCommand(Command command) throws AutoModeEndedException {
        isRunningWithThrow();
        command.start();

        while (isRunningWithThrow() && !command.isFinished()) {
            command.update();
            long waitTime = (long) (update_rate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        command.done();
    }

    public boolean isDone() {
        return hasRun;
    }
}
