package frc2020.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc2020.util.Logger;

/**
 * Gets the current auto mode selected
 */
public class AutoChooser {

    private static Logger logger_ = Logger.getInstance();

    /**
     * All the possible auto modes will go in here
     */
    public enum AutoModeChoices {
        NONE(null);

        public AutoMode autoMode;

        AutoModeChoices(AutoMode autoMode) {
            this.autoMode = autoMode;
        }
    }

    /**
     * Just a getter. Gets our current auto mode. 
     * In the event we don't have one selected 
     * this logs a warning.
     * @param choice current auto mode
     * @return current auto mode
     */
    public static AutoMode getAuto(AutoModeChoices choice) {
        if (choice != null) {
            return choice.autoMode;
        }
        logger_.logWarning("No auto mode selected");
        return null;
    }

    /**
     * Puts out the possible options to be selected for auto
     * @return
     */
    public static SendableChooser<AutoModeChoices> getAutoChooser() {
        SendableChooser<AutoModeChoices> chooser = new SendableChooser<>();
        chooser.setDefaultOption("SELECT AUTO!", null);
        for (AutoModeChoices autoModeChoice : AutoModeChoices.values()) {
            chooser.addOption(autoModeChoice.toString(), autoModeChoice);
        }
        chooser.getSelected();
        return chooser;
    }
}