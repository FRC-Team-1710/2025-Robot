package frc.robot.utils;

import frc.robot.Constants.SimCoralAutomation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SimCoralAutomationChooser {
  private final LoggedDashboardChooser<SimCoralAutomation> simCoralAutomationChooser;

  public SimCoralAutomationChooser() {
    simCoralAutomationChooser = new LoggedDashboardChooser<>("Sim Coral Automation Choises");

    simCoralAutomationChooser.addDefaultOption("Automatically sims coral in robot based on time", SimCoralAutomation.AUTO_SIM_CORAL);
    simCoralAutomationChooser.addOption("Tell the robot the coral state manually", SimCoralAutomation.MANUAL_SIM_CORAL);
  }

  public SimCoralAutomation getAutomationLevel() {
    return simCoralAutomationChooser.get();
  }
}
