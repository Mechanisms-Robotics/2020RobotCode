package frc2020.util.drivers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Creates CTRE motor controller objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class PhoenixFactory {

    private final static int kTimeoutMs = 100;

    /**
     * Create a Talon with an 'out-of-the-box' configuration
     * @param id The CAN bus id of the talon
     * @return The TalonSRX
     */
    public static TalonSRX createDefaultTalon(int id) {
        return createTalon(id);
    }

    /**
     * Create a Victor with an 'out-of-the-box' configuration
     * @param id The CAN bus id of the victor
     * @return The VictorSPX
     */
    public static VictorSPX createDefaultVictor(int id) {
        return createVictor(id);
    }

    /**
     * Create a Talon that is permanently slaved to another master moter controler
     *
     * @param id        ID of the Talon to slave on the CAN bus
     * @param controller_to_follow ID of the controller to slave to
     * @return The slaved Talon
     */
    public static TalonSRX createPermanentSlaveTalon(int id, IMotorController controller_to_follow) {
        final TalonSRX talon = createTalon(id);
        talon.follow(controller_to_follow);
        return talon;
    }

    /**
     * Create a Talon that is permanently slaved to another master talon
     *
     * @param id        ID of the Talon to slave on the CAN bus
     * @param controller_to_follow The controller to follow
     * @return The slaved Talon
     */
    public static VictorSPX createPermanentSlaveVictor(int id, IMotorController controller_to_follow) {
        final VictorSPX victor = createVictor(id);
        victor.follow(controller_to_follow);
        return victor;
    }

    /**
     * Create a Talon from a configuration
     *
     * @param id     The Talon's ID on the CAN bus
     * @return The TalonSRX created
     */
    public static TalonSRX createTalon(int id) {
        TalonSRX talon = new LazyTalonSRX(id);
        talon.set(ControlMode.PercentOutput, 0.0);

        // Sticky faults should be reported and cleared by a previous robot program
        // If they are not handled we want to report them and clear them.
        StickyFaults faults = new StickyFaults();
        talon.getStickyFaults(faults);
        if (faults.hasAnyFault()) {
            StringBuilder builder = new StringBuilder();
            builder.append("Talon " + id + " has uncheck sticky faults");
            builder.append(faults);
            System.out.println(builder.toString());
        }
        talon.clearStickyFaults(kTimeoutMs);
        talon.configFactoryDefault();

        return talon;
    }

    /**
     * Create a Victor from a configuration
     *
     * @param id     The Victors's ID on the CAN bus
     * @return The VictorSPX created
     */
    public static VictorSPX createVictor(int id) {
        VictorSPX victor = new LazyVictorSPX(id);
        victor.set(ControlMode.PercentOutput, 0.0);

        // Sticky faults should be reported and cleared by a previous robot program
        // If they are not handled we want to report them and clear them.
        StickyFaults faults = new StickyFaults();
        victor.getStickyFaults(faults);
        if (faults.hasAnyFault()) {
            StringBuilder builder = new StringBuilder();
            builder.append("Victor " + id + " has uncheck sticky faults");
            builder.append(faults);
            System.out.println(builder.toString());
        }
        victor.clearStickyFaults(kTimeoutMs);
        victor.configFactoryDefault();

        return victor;
    }
}