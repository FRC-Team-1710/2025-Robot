package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class GamePlanChooser {
  private final LoggedDashboardChooser<Boolean> simGamePlanChooser;

  public GamePlanChooser() {
    simGamePlanChooser = new LoggedDashboardChooser<>("Game Plan Choises");

    simGamePlanChooser.addDefaultOption("Points", false);
    simGamePlanChooser.addOption("RP", true);
  }

  public boolean goingForRP() {
    return simGamePlanChooser.get();
  }
}
