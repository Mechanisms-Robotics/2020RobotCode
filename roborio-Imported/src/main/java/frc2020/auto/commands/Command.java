package frc2020.auto.commands;

import frc2020.auto.AutoMode;

/**
 * Command Interface, an interface that describes an iterative action. It is run by an autonomous command, called by the
 * method runAction in AutoMode (or more commonly in autonomous modes that extend AutoMode)
 *
 * @see AutoMode
 */
public interface Command {

    /**
     * Returns whether or not the code has finished execution. When implementing this interface, this method is used by
     * the runCommand method every cycle to know when to stop running the command
     */
    boolean isFinished();

    /**
     * Called by runCommand in AutoMode iteratively until isFinished returns true. Iterative logic lives in this
     * method
     */
    void update();

    /**
     * Run code once when the command finishes, usually for clean up
     */
    void done();

    /**
     * Run code once when the command is started, for set up
     */
    void start();
}
