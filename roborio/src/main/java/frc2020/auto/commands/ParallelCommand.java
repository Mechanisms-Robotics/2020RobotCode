package frc2020.auto.commands;

import java.util.ArrayList;
import java.util.List;

/**
 * Composite command, running all sub-command at the same time. All command are started then updated until all command
 * report being done.
 *
 * parameter A List of Command objects
 */
public class ParallelCommand implements Command {
    private final ArrayList<Command> commands;

    /**
    * Default constructor initializes commands
    */
    public ParallelCommand(List<Command> c) {
        commands = new ArrayList<>(c.size());
        for (Command command : c) {
            commands.add(command);
        }
    }

    /**
    * Returns true if all commands are finished running
    */
    @Override
    public boolean isFinished() {
        boolean all_finished = true;
        for (Command command : commands) {
            if (!command.isFinished()) {
                all_finished = false;
            }
        }
        return all_finished;
    }

    /**
    * Updates every command
    */
    @Override
    public void update() {
        for (Command command : commands) {
            command.update();
        }
    }

    /**
    * Finishes every command
    */
    @Override
    public void done() {
        for (Command command : commands) {
            command.done();
        }
    }

    /**
    * Starts every command
    */
    @Override
    public void start() {
        for (Command command : commands) {
            command.start();
        }
    }
}
