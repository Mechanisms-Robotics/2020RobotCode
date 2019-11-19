package frc2020.auto.commands;

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

    public abstract void runOnce();
}