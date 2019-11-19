package frc2020.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * This class handles everything that needs to iterate
 * Thank you to team 245 for profiding the base for this code!
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

    public Looper() {
        notifier_ = new Notifier(runnable);
        isRunning = false;
        loops_ = new ArrayList<>();
    }

    @Override
    public synchronized void registure(Loop loop) {
        synchronized (syncLock) {
            loops_.add(loop);
        }
    }

    public synchronized void start() {
        if (!isRunning) {
            System.out.println("Starting loops");
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

    public synchronized void stop() {
        if (isRunning) {
            System.out.println("Stopping loops");
            notifier_.stop();
            synchronized (syncLock) {
                isRunning = false;
                for (Loop loop : loops_) {
                    System.out.println("Stopping " + loop);
                    loop.end();
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", dt);
    }
}