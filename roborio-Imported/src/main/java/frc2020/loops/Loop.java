package frc2020.loops;

/**
 * This interface is used by every subsystem.
 * As it extendes the Runnable interface a
 * class that implements it is a thread.
 */
public interface Loop extends Runnable {
    public void init();

    public void run();

    public void end();
}