package frc2020.auto.commands;

import java.util.ArrayList;
import java.util.List;

/**
 * Executes one command at a time. Useful as a member of {@link ParallelCommand}
 */
public class SeriesCommand implements Command {

    private Command curCommand;
    private final ArrayList<Command> remainingCommands;
    private boolean done;

    public SeriesCommand(List<Command> commands) {
        remainingCommands = new ArrayList<>(commands.size());

        for (Command command : commands) {
            remainingCommands.add(command);
        }

        curCommand = null;
        done = false;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void start() {
    }

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
            System.out.println("Finished" + curCommand);
            curCommand.done();
            curCommand = null;
        }
    }

    @Override
    public void done() {
    }
}