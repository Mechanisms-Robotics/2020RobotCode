package frc2020.subsystems;

import org.zeromq.ZContext;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.networking.*;
import frc2020.networking.JetsonMessage.DriveSignal;
import frc2020.networking.JetsonMessage.JetsonUpdate;
import frc2020.robot.Constants;
import frc2020.states.CommandState;
import frc2020.states.CommandState.DriveDemand;
import frc2020.util.Units;
import frc2020.util.geometry.Pose2d;
import frc2020.util.geometry.Rotation2d;

public class Jetson implements Subsystem {
    private final static String JETSON_IP = "10.49.10.26:5556";
    private final static Jetson INSTANCE = new Jetson();

    private final Drive drive_;
    private final DriverStation ds_;

    private final ZContext context_;
    private final ZMQServer server_;
    private final ZMQClient client_;

    private JetsonStatus jetsonStatus_; 
    private RoborioMessage.RioUpdate.Builder rioBuilder_;
    private RoborioMessage.CodeState.Builder codeStateBuilder_;
    private RoborioMessage.WheelOdometryUpdate.Builder wheelOdometryBuilder_;

    private double lastReceivedTimestamp_;
    
    private Jetson() {
        context_ = new ZContext();
        server_ = new ZMQServer(context_);
        server_.connect();
        client_ = new ZMQClient(context_, JETSON_IP);
        client_.connect();
        drive_ = Drive.getInstance();
        ds_ = DriverStation.getInstance();
        jetsonStatus_ = new JetsonStatus();
        rioBuilder_ = RoborioMessage.RioUpdate.newBuilder();
        codeStateBuilder_ = RoborioMessage.CodeState.newBuilder();
        wheelOdometryBuilder_ = RoborioMessage.WheelOdometryUpdate.newBuilder();
        lastReceivedTimestamp_ = -1.0;
        SmartDashboard.putBoolean("Pre-Start", false);
    }

    public static Jetson getInstance() {
        return INSTANCE;
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO: Publish values from ZMQServer
        int codeStateInt;
        if (ds_.isDisabled()) {
            codeStateInt = 0;
        } else if (ds_.isAutonomous()) {
            codeStateInt = 1;
        } else {
            codeStateInt = 2;
        }
        
        codeStateBuilder_.setEnabledStateValue(codeStateInt);
        codeStateBuilder_.setPreStart(SmartDashboard.getBoolean("Pre-Start", false));
        
        double left = Units.inches_to_meters(drive_.getLeftLinearVelocity());
        double right = Units.inches_to_meters(drive_.getRightLinearVelocity());
        wheelOdometryBuilder_.setLeft((float)left);
        wheelOdometryBuilder_.setRight((float)right);
        wheelOdometryBuilder_.setTimeStamp((float)Timer.getFPGATimestamp());

        rioBuilder_.setCodeState(codeStateBuilder_);
        rioBuilder_.setOdometryUpdate(wheelOdometryBuilder_);
        server_.publishUpdate(rioBuilder_.build());

        rioBuilder_.clear();
        codeStateBuilder_.clear();
        wheelOdometryBuilder_.clear();
    }

    @Override
    public void readPeriodicInputs() {
        // TODO: Read in values from ZMQClient
        // Should write to jetsonStatus_
        JetsonUpdate update = client_.getUpdate(true);
        if (update != null) {
            lastReceivedTimestamp_ = Timer.getFPGATimestamp();
            if (update.getDriveSignal() != null) {
                frc2020.util.DriveSignal signal;
                double left = update.getDriveSignal().getDemandLeft();
                double right = update.getDriveSignal().getDemandRight();
                signal = new frc2020.util.DriveSignal(left, right);
                int type = update.getDriveSignal().getDemandTypeValue();

                jetsonStatus_.driveDemand = new DriveDemand(signal, DriveDemand.fromDSType(type));
            }
            if (update.getSlamUpdate() != null) {
                double x = update.getSlamUpdate().getX();
                double y = update.getSlamUpdate().getY();
                double theta = update.getSlamUpdate().getTheta();
                double timestamp = update.getSlamUpdate().getUpdateEpoc();

                jetsonStatus_.pose = new Pose2d(x, y, Rotation2d.fromRadians(theta));
                jetsonStatus_.slamTimestamp = timestamp;
            }
        }
    }

    // Basicly getters for everthing in jetsonStatus_ would go here.

    public class JetsonStatus {
        public Pose2d pose = new Pose2d();
        public double slamTimestamp = -1.0;
        public CommandState.DriveDemand driveDemand 
            = new CommandState.DriveDemand(new frc2020.util.DriveSignal(0, 0));
    }

    public JetsonStatus getJetsonStatus() {
        return jetsonStatus_;
    }

    public boolean hasReceivedRecentDemand() {
        if((Timer.getFPGATimestamp() - lastReceivedTimestamp_) >= Constants.LAST_RECIEVED_MESSAGE_TIMEOUT/1000){
            return false;
        } 
        return true;
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

    public double getLastReceivedTimestamp_() {
        return lastReceivedTimestamp_;
    }

    @Override
    public void outputTelemetry() {
        // Put vaules we are getting from the network here (Such as robot pose. ect.)
        SmartDashboard.putNumber("Jetson X", jetsonStatus_.pose.getTranslation().x());
        SmartDashboard.putNumber("Jetson Y", jetsonStatus_.pose.getTranslation().y());
        SmartDashboard.putNumber("Jetson Theta", jetsonStatus_.pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Jetson Timestamp", jetsonStatus_.slamTimestamp);
        SmartDashboard.putNumber("Jetson Drive Left", jetsonStatus_.driveDemand.demand.getLeft());
        SmartDashboard.putNumber("Jetson Drive Right", jetsonStatus_.driveDemand.demand.getRight());
        SmartDashboard.putString("Jetson Demand Type", jetsonStatus_.driveDemand.type.toString());
    }
}