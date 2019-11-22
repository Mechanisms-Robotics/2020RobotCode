package frc2020.auto;

import frc2020.auto.commands.Command;

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

    public void run() {
        running = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            System.out.print("WARNING Auto ended and we weren't done!");
        }

        done();
        System.out.println("Auto mode done");
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