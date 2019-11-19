package frc2020.auto;

import frc2020.loops.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous mode.
 *
 * @author Team 254
 * @author Modified by Team 4910
 */
public class AutoModeRunner {
    private AutoMode auto_mode;
    private Thread thread = null;

    /*
     * Sets what auto mode to run.
     * @param new_auto_mode The auto mode to ready
     */
    public void setAutoMode(AutoMode new_auto_mode) {
        auto_mode = new_auto_mode;
    }

    /*
     * Run the selected auto mode
     */
    public void start() {
        if (thread == null) {
            thread = new Thread(new CrashTrackingRunnable() {
                @Override
                public void runCrashTracked() {
                    if (auto_mode != null) {
                        auto_mode.run();
                    }
                }
            });

            thread.start();
        }

    }

    /*
     * Stop the currently running auto mode.
     */
    public void stop() {
        if (auto_mode != null) {
            auto_mode.stop();
        }

        thread = null;
    }

}
