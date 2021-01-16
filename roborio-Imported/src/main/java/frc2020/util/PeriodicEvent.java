package frc2020.util;

/**
 * Interface for an event that is to be run periodically
 */
public interface PeriodicEvent {

    /**
     * @return Whether to call run or not
     */
    public boolean condition();

    /**
     * Method to run periodically 
     */
    public void run();

};