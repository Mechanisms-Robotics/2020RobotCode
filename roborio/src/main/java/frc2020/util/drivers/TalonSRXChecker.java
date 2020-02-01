package frc2020.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc2020.subsystems.Subsystem;
import frc2020.util.Logger;
import frc2020.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class TalonSRXChecker {

    private static Logger logger_ = Logger.getInstance();

    public static class CheckerConfig {
        public double mCurrentFloor = 5;
        public double mRPMFloor = 2000;

        public double mCurrentEpsilon = 5.0;
        public double mRPMEpsilon = 500;
        public DoubleSupplier mRPMSupplier = null;

        public double mRunTimeSec = 4.0;
        public double mWaitTimeSec = 2.0;
        public double mRunOutputPercentage = 0.5;
    }

    public static class TalonSRXConfig {
        public String mName;
        public TalonSRX mTalon;

        public TalonSRXConfig(String name, TalonSRX talon) {
            mName = name;
            mTalon = talon;
        }
    }

    private static class StoredTalonSRXConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    public static boolean CheckTalons(Subsystem subsystem,
                                      ArrayList<TalonSRXConfig> talonsToCheck,
                                      CheckerConfig checkerConfig) {
        boolean failure = false;
        logger_.logInfo("Starting Talon Checks...");
        logger_.logInfo("Checking subsystem " + subsystem.getClass()
                + " for " + talonsToCheck.size() + " talons.");
        
        String logName = subsystem.getClass().getName();

        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();
        ArrayList<StoredTalonSRXConfiguration> storedConfigurations = new ArrayList<>();

        // Record previous configuration for all talons.
        for (TalonSRXConfig config : talonsToCheck) {
            LazyTalonSRX talon = LazyTalonSRX.class.cast(config.mTalon);

            StoredTalonSRXConfiguration configuration = new StoredTalonSRXConfiguration();
            configuration.mMode = talon.getControlMode();
            configuration.mSetValue = talon.getLastSet();

            storedConfigurations.add(configuration);

            // Now set to disabled.
            talon.set(ControlMode.PercentOutput, 0.0);
        }

        for (TalonSRXConfig config : talonsToCheck) {
            logger_.logInfo("Checking " + config.mName, logName);

            config.mTalon.set(ControlMode.PercentOutput, checkerConfig.mRunOutputPercentage);
            Timer.delay(checkerConfig.mRunTimeSec);

            // Now poll the interesting information.
            double current = config.mTalon.getStatorCurrent();
            currents.add(current);

            logger_.logDebug("Current " + current, logName);
            double rpm = Double.NaN;
            if (checkerConfig.mRPMSupplier != null) {
                rpm = checkerConfig.mRPMSupplier.getAsDouble();
                rpms.add(rpm);
                logger_.logDebug("RPM " + rpm, logName);
            }

            config.mTalon.set(ControlMode.PercentOutput, 0.0);

            // And perform checks.
            if (current < checkerConfig.mCurrentFloor) {
                logger_.logWarning(config.mName
                        + " has failed current floor check vs " +
                        checkerConfig.mCurrentFloor + "!!", logName);
                failure = true;
            } else {
                logger_.logDebug(config.mName + " has passed current test", logName);
            }
            if (checkerConfig.mRPMSupplier != null) {
                if (rpm < checkerConfig.mRPMFloor) {
                    logger_.logWarning(config.mName
                            + " has failed rpm floor check vs " +
                            checkerConfig.mRPMFloor + "!!", logName);
                    failure = true;
                } else {
                    logger_.logDebug(config.mName + " has passed rpm test", logName);
                }
            }

            Timer.delay(checkerConfig.mWaitTimeSec);
        }

        // Now run aggregate checks.

        if (currents.size() > 0) {
            Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(currents, average, checkerConfig.mCurrentEpsilon)) {
                logger_.logWarning("Currents varied", logName);
                failure = true;
            }
        }

        if (rpms.size() > 0) {
            Double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(rpms, average, checkerConfig.mRPMEpsilon)) {
                logger_.logWarning("RPMs  varied", logName);
                failure = true;
            }
        }

        // Restore Talon configurations
        for (int i = 0; i < talonsToCheck.size(); ++i) {
            talonsToCheck.get(i).mTalon.set(storedConfigurations.get(i).mMode,
                    storedConfigurations.get(i).mSetValue);
        }

        return !failure;
    }
}