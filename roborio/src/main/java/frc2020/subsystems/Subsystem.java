package frc2020.subsystems;

import frc2020.loops.ILooper;

/**
* The interface that all subsystems implement.
*
* @author Team 4910
*/
public interface Subsystem {

    static Subsystem getInstance(){
        return null;
    };

    /**
     * Handles any periodic writing any outputs to the seperate subsystems themselves.
     */
    void writePeriodicOutputs();

    /**
     * Periodically reads all of the data that can be gotten from that specific subsystem.
     */
    void readPeriodicInputs();

    /**
     * Runs the non-movement-based tests for each subsystem
     * @return true if all tests passed
     */
    boolean runPassiveTests();

    /**
     * Runs the movement-based tests for each subsystem
     * @return true if all tests passed
     */
    boolean runActiveTests();

    /**
     * Resets all the sensors of that subsystem to their native positions
     */
    void zeroSensors();

    /**
     * Handles how the subsystem would stop (Usually setting motors to 0 but sometimes
     * extra steps need to be taken i.e. neutral mode for drive)
     */
    void stop();

    /**
     * Registers all loops needed to run for this subsystem
     * 
     * @param enabledLooper used to register an Enabled loop in subsystem manager
     */
    void registerLoops(ILooper enabledLooper);

    /**
     * Outputs all the data to the Smart Dashboard
     */
    void outputTelemetry();
}
