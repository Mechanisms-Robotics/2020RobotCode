package frc2020.auto.commands;

import java.util.ArrayList;
import java.util.List;

import frc2020.util.Logger;

/**
 * Executes one command at a time. Useful as a member of {@link ParallelCommand}
 */
public class SeriesCommand implements Command {

    private Command curCommand;
    private final ArrayList<Command> remainingCommands;
    private boolean done;

    private static Logger logger_ = Logger.getInstance();

    /**
    * Default constructor, initializes remainingCommands
    */
    public SeriesCommand(List<Command> commands) {
        remainingCommands = new ArrayList<>(commands.size());

        for (Command command : commands) {
            remainingCommands.add(command);
        }

        curCommand = null;
        done = false;
    }

    /**
    * Returns done
    */
    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void start() {
    }

    /**
    * Will start/update curCommand until it is finished then move on to the next
    */
    @Override
    public void update() {
        if (curCommand == null) {
            if (remainingCommands.isEmpty()) {
                done = true;
                return;
            }

            curCommand = remainingCommands.remove(0);
            curCommand.start();
        }

        curCommand.update();

        if (curCommand.isFinished()) {
            logger_.logDebug("Finished" + curCommand);
            curCommand.done();
            curCommand = null;
        }
    }

    @Override
    public void done() {
    }
}
