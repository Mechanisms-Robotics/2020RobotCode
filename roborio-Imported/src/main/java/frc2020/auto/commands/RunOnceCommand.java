package frc2020.auto.commands;

/**
 * Abstract Command class that should be run only once
 */
public abstract class RunOnceCommand implements Command {
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public void start() {
    }

    @Override
    public void done() {
        runOnce();
    }

    /**
     * Overide this function and input the command that
     * you want to be run only once
     */
    public abstract void runOnce();
}