package frc2020.auto.commands;

import java.util.ArrayList;
import java.util.List;

/**
 * Composite command, running all sub-command at the same time All command are started then updated until all command
 * report being done.
 *
 * @param A List of Command objects
 */
public class ParallelCommand implements Command {
    private final ArrayList<Command> commands;

    public ParallelCommand(List<Command> c) {
        commands = new ArrayList<>(c.size());
        for (Command command : c) {
            commands.add(command);
        }
    }

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

    @Override
    public void update() {
        for (Command command : commands) {
            command.update();
        }
    }

    @Override
    public void done() {
        for (Command command : commands) {
            command.done();
        }
    }

    @Override
    public void start() {
        for (Command command : commands) {
            command.start();
        }
    }
}