package frc2020.util;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * Manages running all PeriodicEvents periodically
 */
public class PeriodicEventManager {

    private List<PeriodicEvent> events_ = new ArrayList<PeriodicEvent>();
    private List<Double> eventIntervals_ = new ArrayList<Double>();
    private List<Double> lastTimes_ = new ArrayList<Double>();

    /**
     * @param event Event to run periodically
     * @param eventInterval Periodic interval to run event
     */
    public void addEvent(PeriodicEvent event, double eventInterval) {
        events_.add(event);
        eventIntervals_.add(eventInterval);
        lastTimes_.add(Timer.getFPGATimestamp());
    }

    /**
     * @param event Event to remove
     */
    public void removeEvent(PeriodicEvent event) {
        for (int i = 0; i < events_.size(); i++) {
            if (events_.get(i) == event) {
                events_.remove(i);
                eventIntervals_.remove(i);
                lastTimes_.remove(i);
            }
        }
    }

    /**
     * Handles periodic calling of all PeriodicEvent's code
     */
    public void run() {
        for (int i = 0; i < events_.size(); i++){
            if (Timer.getFPGATimestamp()-lastTimes_.get(i) >= eventIntervals_.get(i)) {
                if (events_.get(i).condition()){
                    events_.get(i).run();
                }
                lastTimes_.set(i, Timer.getFPGATimestamp());
            }
        }
    }

};