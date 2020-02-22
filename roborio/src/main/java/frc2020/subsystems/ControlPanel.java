package frc2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2020.loops.ILooper;
import frc2020.loops.Loop;
import frc2020.util.WheelWatcher;
import frc2020.util.WheelWatcher.WheelColor;

public class ControlPanel extends SingleMotorSubsystem {
    private static ControlPanel instance_;

    //TODO: change on actual robot
    private final static int FLIPPER_FORWARD_PORT = 0; 
    private final static int FLIPPER_REVERSE_PORT = 1;
    private final static DoubleSolenoid.Value STOWED_VALUE = Value.kReverse;
    private final static DoubleSolenoid.Value DEPLOYED_VALUE = Value.kForward;
    private final static double FORWARD_RPM = 3000;
    private final static double REVERSE_RPM = -3000;
    private final static double GOAL_EDGE_COUNT = 3.0*8.0; // Three rotations | Eight edge counts per rotation
    private final static String gameData = null;

    private WheelWatcher wheelWatcher_ = new WheelWatcher();

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
        new SingleMotorSubsystemConstants();

    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 6; //TODO: change on actual robot
        masterConstants.invertMotor_ = false;
        
        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.name_ = "ControlPanel";
    }

    private DoubleSolenoid flipper_;
    private boolean wantDeploy_ = false;
    private boolean isDeployed_ = false;

    private ControlPanelState state_ = ControlPanelState.IDLE;

    private WheelColor goalColor_ = WheelColor.UNKNOWN;

    private static enum ControlPanelState {
        IDLE, POSITION, ROTATION, MANUAL
    };

    public static ControlPanel getInstance() {
        return instance_ == null ? instance_ = new ControlPanel(DEFAULT_CONSTANTS) : instance_;
    }

    protected ControlPanel(SingleMotorSubsystemConstants constants) {
        super(constants);
        
        flipper_ = new DoubleSolenoid(FLIPPER_FORWARD_PORT, FLIPPER_REVERSE_PORT);
    }

    public void deployPanelArm() {
        wantDeploy_ = true;
    }

    public void stowPanelArm() {
        state_ = ControlPanelState.IDLE;
        super.stop();
        wantDeploy_ = false;
    }

    public void togglePanelArm() {
        wantDeploy_ = !wantDeploy_;
        if (!wantDeploy_) {
            super.stop();
        }
    }

    public void runPanelWheel(boolean reverse) {
        if (reverse) {
            super.setVelocity(REVERSE_RPM);
        }
        else {
            super.setVelocity(FORWARD_RPM);
        }
    }

    @Override
    public boolean runActiveTests() {
        logger_.logInfo("Running panel arm active tests", super.logName_);

        logger_.logInfo("Deploying panel arm", super.logName_);
        deployPanelArm();
        Timer.delay(1.5);

        logger_.logInfo("Spinning panel wheel in forward direction", super.logName_);
        runPanelWheel(false);
        Timer.delay(1.5);
        
        logger_.logInfo("Spinning panel wheel in reverse direction", super.logName_);
        runPanelWheel(true);
        Timer.delay(1.5);
        
        logger_.logInfo("Stowing panel arm", super.logName_);
        stowPanelArm();

        return true;
    }

    @Override
    public void zeroSensors() {
        wheelWatcher_.reset();
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
        enabledLooper.register(controlPanelLoop);
    }

    private final Loop controlPanelLoop = new Loop() {

        ControlPanelState lastState_ = ControlPanelState.IDLE;

        public void init() {
            synchronized (ControlPanel.this) {
                state_ = ControlPanelState.IDLE;
                wheelWatcher_.init();
            }
        }

        public void run() {
            synchronized (ControlPanel.this) {
                if (state_ != lastState_) {
                    logger_.logInfo("Control Panel state changed to "+state_.toString(), logName_);

                    lastState_ = state_;
                }

                switch(state_) {
                    case IDLE:
                        break;
                    case POSITION:
                        wheelWatcher_.update();

                        if (goalColor_ == WheelColor.UNKNOWN) {
                            state_ = ControlPanelState.IDLE;
                            logger_.logWarning("Control Panel unknown goal color", logName_);
                            break;
                        }

                        WheelColor colorCompliment = wheelWatcher_.getColorAt90(goalColor_);

                        if (colorCompliment == goalColor_) {
                            state_ = ControlPanelState.IDLE;
                            stop();
                            break;
                        }

                        runPanelWheel(true);
                        break;
                    case ROTATION:
                        wheelWatcher_.update();

                        if (wheelWatcher_.getEdgeCount() == GOAL_EDGE_COUNT) {
                            state_ = ControlPanelState.IDLE;
                            stop();
                            break;
                        }

                        runPanelWheel(true);
                        break;
                    case MANUAL:
                        break;
                    default:
                        logger_.logWarning("Control Panel unexpected state "+state_.toString(), logName_);
                        break;
                }
            }
        }

        public void end() {
            synchronized (ControlPanel.this) {
                state_ = ControlPanelState.IDLE;
                stop();
            }
        }

    };


    public synchronized void startPositionControl() {
        wheelWatcher_.reset();
        goalColor_ = getFMSColor(gameData);
        state_ = ControlPanelState.POSITION;
    }

    public synchronized void startRotationControl() {
        wheelWatcher_.reset();
        state_ = ControlPanelState.ROTATION;
    }

    public synchronized void startManualControl() {
        if (state_ != ControlPanelState.MANUAL) {
            state_ = ControlPanelState.MANUAL;
        }
    }

    public static WheelColor getFMSColor(String gameData) {
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        gameData = gameData.toUpperCase();
        if (gameData.length() > 0) {
           switch (gameData.charAt(0)) {
               case 'R': return WheelColor.RED;
               case 'B': return WheelColor.BLUE;
               case 'G': return WheelColor.GREEN;
               case 'Y': return WheelColor.YELLOW;
           }
        }
        return WheelColor.UNKNOWN;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Panel Arm deployed", isDeployed_);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();

        if (wantDeploy_ != isDeployed_) {
            if (wantDeploy_) {
                flipper_.set(DEPLOYED_VALUE);
            } else {
                flipper_.set(STOWED_VALUE);
            }
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();

        isDeployed_ = flipper_.get() == DEPLOYED_VALUE;
    }

    @Override
    protected boolean atReverseLimit() {
        return !isDeployed_;
    }

    @Override
    protected boolean atForwardLimit() {
        return !isDeployed_;
    }

    @Override
    protected boolean handleZeroing() {
        return true;
    }
}
