package frc.robot.utils;

import frc.robot.Constants.AutomationLevel;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutomationLevelChooser {
  private final LoggedDashboardChooser<AutomationLevel> automationLevelChooser;

  public AutomationLevelChooser() {
    automationLevelChooser = new LoggedDashboardChooser<>("Automation Choises");

    automationLevelChooser.addOption("Auto Drive", AutomationLevel.AUTO_DRIVE);
    automationLevelChooser.addOption("No Auto Drive", AutomationLevel.NO_AUTO_DRIVE);
  }

  public AutomationLevel getAutomationLevel() {
    return automationLevelChooser.get();
  }
}
