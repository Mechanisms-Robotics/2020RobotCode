package frc2020.subsystems;

import frc2020.loops.Loop;
import frc2020.loops.Looper;
import frc2020.loops.ILooper;

import java.util.ArrayList;
import java.util.List;

public class SubsystemManager implements ILooper {

    private final List<Subsystem> allSubsystems;
    private List<Loop> loops_ = new ArrayList<>();

    public SubsystemManager(List<Subsystem> subsystems) {
        allSubsystems = subsystems;
    }

    public void outputToSmartDashboard() {
        allSubsystems.forEach((s) -> s.outputTelemetry());
    }

    public void outputToLog() {
        allSubsystems.forEach((s) -> s.writeToLog());
    }

    public void stop() {
        allSubsystems.forEach((s) -> s.stop());
    }

    private class Enabled implements Loop {

        @Override
        public void init() {
            for(Loop i : loops_){
                i.init();
            }
        }

        @Override
        public void run() {
            for(Subsystem s : allSubsystems){
                s.readPeriodicInputs();
            }
            for(Loop i : loops_) {
                i.run();
            }
            for(Subsystem s : allSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void end() {
            for (Loop i : loops_) {
                i.end();
            }
        }
    }

    private class Disabled implements Loop {

        @Override
        public void init() {

        }

        @Override
        public void run() {
            for (Subsystem s : allSubsystems) {
                s.readPeriodicInputs();
            }
            for (Subsystem s : allSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void end() {

        }
    }

    public void registerEnabledLoops(Looper loop) {
        allSubsystems.forEach((s) -> s.registerLoops(this));
        loop.registure(new Enabled());
    }

    public void registerDisabledLoops(Looper loop) {
        loop.registure(new Disabled());
    }

    @Override
    public void registure(Loop loop) {
        loops_.add(loop);
    }
}