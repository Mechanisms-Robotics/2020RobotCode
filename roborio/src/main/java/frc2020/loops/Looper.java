package frc2020.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.util.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * This class handles everything that needs to iterate
 * Thank you to team 254 for providing the base for this code!
 * Original Code:
 * Copyright (c) 2017 Team 254
 */
public class Looper implements ILooper {
    public final static double kPeriod = 0.01;

    private boolean isRunning;

    private final Notifier notifier_;
    private final List<Loop> loops_;
    private final Object syncLock = new Object();
    private double timestamp = 0;
    private double dt = 0;

    private static Logger logger_ = Logger.getInstance();

    /**
    * This is the runnable object that the notifier_ will take in and call
    * runCrashTracked() every 10 miliseconds
    */
    private final CrashTrackingRunnable runnable = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized (syncLock) {
                if (isRunning) {
                    double now = Timer.getFPGATimestamp();
                    for (Loop loop : loops_) {
                        loop.run();
                    }
                    dt = now - timestamp;
                    timestamp = now;
                }
            }
        }
    };

    /**
    * Default constructor for Looper will initialize the notifier and loops
    */
    public Looper() {
        notifier_ = new Notifier(runnable);
        isRunning = false;
        loops_ = new ArrayList<>();
    }

    /**
    * Will register a new loop.
    */
    @Override
    public synchronized void register(Loop loop) {
        synchronized (syncLock) {
            loops_.add(loop);
        }
    }

    /**
    * Will run init() on all loops and start the notifier's periodic loop
    */
    public synchronized void start() {
        if (!isRunning) {
            logger_.logDebug("Starting loops");
            synchronized (syncLock) {
                timestamp = Timer.getFPGATimestamp();
                for (Loop loop : loops_) {
                    loop.init();
                }
                isRunning = true;
            }
            notifier_.startPeriodic(kPeriod);
        }
    }

    /**
    * Will run end() on all loops and stop the notifier's periodic loop
    */
    public synchronized void stop() {
        if (isRunning) {
            logger_.logDebug("Stopping loops");
            notifier_.stop();
            synchronized (syncLock) {
                isRunning = false;
                for (Loop loop : loops_) {
                    logger_.logDebug("Stopping " + loop);
                    loop.end();
                }
            }
        }
    }

    /**
    * Will output the delta_time to SmartDashboard
    */
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", dt);
    }
}
