package frc2020.subsystems;

import frc2020.loops.Loop;
import frc2020.loops.Looper;
import frc2020.loops.ILooper;

import java.util.ArrayList;
import java.util.List;

/**
* The manager of all subsystems on the robot. It makes sure that they run
* periodically and output to SmartDashboard.
*
* @author Team 4910
*/
public class SubsystemManager implements ILooper {

    private final List<Subsystem> allSubsystems;
    private List<Loop> loops_ = new ArrayList<>();

    /**
     * The default constructor for the SubsystemManager instantiates it with
     * a list of subsystems
     */
    public SubsystemManager(List<Subsystem> subsystems) {
        allSubsystems = subsystems;
    }

    /**
    * Makes every subsystem output it's telemetry to SmartDashboard
    */
    public void outputToSmartDashboard() {
        allSubsystems.forEach((s) -> s.outputTelemetry());
    }

    /**
    * Makes every subsystem output it's telemetry to a csv
    */
    public void outputToLog() {
        allSubsystems.forEach((s) -> s.writeToLog());
    }

    /**
    * Stops every subsystem
    */
    public void stop() {
        allSubsystems.forEach((s) -> s.stop());
    }

    /**
    * This class implements a periodic loop that will run every subsystem's
    * loop aswell as read and write data when the robot is enabled.
    */
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

    /**
    * This class implements a periodic loop that will read and write data
    * from every subsystem when the robot is disabled.
    */
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

    /**
    * Will register each subsystem's loop aswell as an Enabled loop
    */
    public void registerEnabledLoops(Looper loop) {
        allSubsystems.forEach((s) -> s.registerLoops(this));
        loop.register(new Enabled());
    }

    /**
    * Will register a Disabled loop
    */
    public void registerDisabledLoops(Looper loop) {
        loop.register(new Disabled());
    }

    /**
    * Will register a loop
    */
    @Override
    public void register(Loop loop) {
        loops_.add(loop);
    }
}
