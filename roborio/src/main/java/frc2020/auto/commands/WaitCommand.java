package frc2020.auto.commands;

import edu.wpi.first.wpilibj.Timer;

/**
 * Command to wait for a given amount of time To use this Command, call runCommand(new WaitCommand(your_time))
 */
public class WaitCommand implements Command {

    private double timeToWait;
    private double startTime;

    /**
    * Default construtor, initializes timeToWait
    */
    public WaitCommand(double time) {
        timeToWait = time;
    }

    /**
    * Will return true if has waited timeToWait
    */
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

    /**
    * Initializes startTime
    */
    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
}
