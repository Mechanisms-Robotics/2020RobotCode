package frc2020.auto.commands;

import edu.wpi.first.wpilibj.Timer;

/**
 * Command to wait for a given amount of time To use this Command, call runCommand(new WaitCommand(your_time))
 */
public class WaitCommand implements Command {

    private double timeToWait;
    private double startTime;

    public WaitCommand(double time) {
        timeToWait = time;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
}