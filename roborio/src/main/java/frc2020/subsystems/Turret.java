package frc2020.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc2020.loops.ILooper;

public class Turret extends SingleMotorSubsystem {

    private static Turret instance_;

    private final static SingleMotorSubsystemConstants DEFAULT_CONSTANTS =
            new SingleMotorSubsystemConstants();
    static {
        var masterConstants = new MotorConstants();
        masterConstants.id_ = 7;
        masterConstants.invertMotor_ = false;
        masterConstants.invertSensorPhase_ = false;

        DEFAULT_CONSTANTS.masterConstants_ = masterConstants;
        DEFAULT_CONSTANTS.homePosition_ = 0.0; //degrees
        DEFAULT_CONSTANTS.name_ = "Turret";
        DEFAULT_CONSTANTS.enableHardLimits_ = false;

        DEFAULT_CONSTANTS.positionConversionFactor_ = 3.3; // degrees / encoder
        DEFAULT_CONSTANTS.velocityConversionFactor_ = 3.3;
        DEFAULT_CONSTANTS.closedLoopRampRate_ = 0.1;
        DEFAULT_CONSTANTS.cruiseVelocity_ = 0; //deg/min
        DEFAULT_CONSTANTS.acceleration_ = 0; //deg/min^2
        DEFAULT_CONSTANTS.velocityDeadBand_ = 2.5;

        DEFAULT_CONSTANTS.kP_ = 0.0001;
        DEFAULT_CONSTANTS.kI_ = 0.0;
        DEFAULT_CONSTANTS.kD_ = 0.0;
        DEFAULT_CONSTANTS.kF_ = 0.0;
    }

    private final static Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(180);

    public static Turret getInstance() {
        return instance_ == null ? instance_ = new Turret(DEFAULT_CONSTANTS) : instance_;
    }

    protected Turret(SingleMotorSubsystemConstants constants) {
        super(constants);
    }

    @Override
    protected boolean handleZeroing() {
        return false;
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected synchronized boolean atReverseLimit() {
        return false;
    }

    // TODO: Make sure sensor phase is correct (clockwise negative)
    @Override
    protected synchronized boolean atForwardLimit() {
        return false;
    }

    @Override
    public boolean runActiveTests() {
        return true;
    }

    @Override
    public void zeroSensors() {
        encoder.setPosition(0.0);
    }

    @Override
    public void registerLoops(ILooper enabledLooper) {
    }
}
