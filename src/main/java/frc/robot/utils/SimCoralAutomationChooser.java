package frc.robot.utils;

import frc.robot.Constants.SimCoralAutomation;

public class SimCoralAutomationChooser {
  // private final LoggedDashboardChooser<SimCoralAutomation> simCoralAutomationChooser;

  public SimCoralAutomationChooser() {
    // simCoralAutomationChooser = new LoggedDashboardChooser<>("Sim Coral Automation Choices");

    // simCoralAutomationChooser.addDefaultOption(
    //     "Tell the robot the coral state manually", SimCoralAutomation.MANUAL_SIM_CORAL);
    // simCoralAutomationChooser.addOption(
    //     "Automatically sims coral in robot based on time", SimCoralAutomation.AUTO_SIM_CORAL);
  }

  public SimCoralAutomation getAutomationLevel() {
    return SimCoralAutomation.MANUAL_SIM_CORAL;
  }
}
