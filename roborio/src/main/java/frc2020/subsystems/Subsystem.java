package frc2020.subsystems;

import frc2020.loops.ILooper;

public interface Subsystem {

    static Subsystem getInstance(){
        return null;
    };

    void writePeriodicOutputs();
    void readPeriodicInputs();
    boolean checkSystem();
    void zeroSensors();

    void stop();
    void registerLoops(ILooper enabledLooper);

    void writeToLog();
    void outputTelemetry();
}