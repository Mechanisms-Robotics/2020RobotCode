package frc2020.auto.commands;

import frc2020.subsystems.Intake;

public class IntakeCommand implements Command {

    private Intake intake_ = Intake.getInstance();

    public IntakeCommand(boolean deploy) {
        if (deploy) {
            intake_.deployIntake();
            intake_.runIntakeAuto(false);
        } else {
            intake_.stowIntake();
            intake_.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
    }
    
}
