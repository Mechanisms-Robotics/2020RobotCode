package frc2020.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChooser {
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
        System.out.println("No auto mode selected");
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