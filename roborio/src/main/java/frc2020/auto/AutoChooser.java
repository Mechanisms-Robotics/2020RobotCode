package frc2020.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc2020.util.Logger;

public class AutoChooser {
    private static Logger logger_ = Logger.getInstance();

    public enum AutoModeChoices {
        NONE(null);

        public AutoMode autoMode;

        AutoModeChoices(AutoMode autoMode) {
            this.autoMode = autoMode;
        }
    }

    public static AutoMode getAuto(AutoModeChoices choice) {
        if (choice != null) {
            return choice.autoMode;
        }
        logger_.logWarning("No auto mode selected");
        return null;
    }

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