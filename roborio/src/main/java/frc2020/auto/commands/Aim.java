package frc2020.auto.commands;

import frc2020.subsystems.Shooter;
import frc2020.subsystems.Shooter.ShooterState;

public class Aim implements Command {
    private static Shooter shooter_ = Shooter.getInstance();

    @Override
    public boolean isFinished() { return true; }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {
        shooter_.setState(ShooterState.Aiming);
    }
}
