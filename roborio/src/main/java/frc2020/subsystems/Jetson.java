package frc2020.subsystems;

import org.zeromq.ZContext;

import edu.wpi.first.wpilibj.DriverStation;
import frc2020.loops.ILooper;
import frc2020.networking.ZMQClient;
import frc2020.networking.ZMQServer;

class Jetson implements Subsystem {
    private final static String JETSON_IP = "10.49.10.26:5555";
    private final static Jetson INSTANCE = new Jetson();

    private final Drive drive_;
    private final DriverStation ds_;

    private final ZContext context_;
    private final ZMQServer server_;
    private final ZMQClient client_;

    private JetsonStatus jetsonStatus_; 
    
    private Jetson() {
        context_ = new ZContext();
        server_ = new ZMQServer(context_);
        server_.connect();
        client_ = new ZMQClient(context_, JETSON_IP);
        client_.connect();
        drive_ = Drive.getInstance();
        ds_ = DriverStation.getInstance();
        jetsonStatus_ = new JetsonStatus();
    }

    public static Jetson getInstance() {
        return INSTANCE;
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO: Publish values from ZMQServer
    }

    @Override
    public void readPeriodicInputs() {
        // TODO: Read in values from ZMQClient
        // Should write to jetsonStatus_
    }

    // Basicly getters for everthing in jetsonStatus_ would go here.

    private class JetsonStatus {
        // Define the current recvied status of the jetson here
        // should include timestamp on the message that the jetson had
        // and the timestamp in robot seconds (Timer.getFPGATimestamp())
        // Other things could include pose, auto setpounts, vision targets
        // ect.
    }

    @Override
    public boolean checkSystem() {
        // Not exactly sure what to do here...
        return true;
    }

    @Override
    public void zeroSensors() {
        // I think that the jetson should handle this but mabey 
        // there is a use for this
    }

    @Override
    public void stop() {
        // Nothing to do here as there is no hardware to stop
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        // I don't think we have any spcific things we want to do
        // just when we are enabled but I could be wrong.
    }

    @Override
    public void outputTelemetry() {
        // Put vaules we are getting from the network here (Such as robot pose. ect.)
    }
}