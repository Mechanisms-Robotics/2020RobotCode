package frc2020.util;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class PeriodicEventManager {

    private List<PeriodicEvent> events_ = new ArrayList<PeriodicEvent>();
    private List<Double> eventIntervals_ = new ArrayList<Double>();
    
    private double lastTime_ = 0.0;

    public PeriodicEventManager() {
        this.lastTime_ = Timer.getFPGATimestamp();
    }

    public void addEvent(PeriodicEvent event, double eventInterval) {
        events_.add(event);
        eventIntervals_.add(eventInterval);
    }

    public void removeEvent(PeriodicEvent event) {
        for (int i = 0; i < events_.size(); i++) {
            if (events_.get(i) == event) {
                events_.remove(i);
                eventIntervals_.remove(i);
            }
        }
    }

    public void run() {
        for (int i = 0; i < events_.size(); i++){
            if (Timer.getFPGATimestamp()-this.lastTime_ >= eventIntervals_.get(i)) {
                if (events_.get(i).condition()){
                    events_.get(i).run();
                }
                this.lastTime_ = Timer.getFPGATimestamp();
            }
        }
    }

};