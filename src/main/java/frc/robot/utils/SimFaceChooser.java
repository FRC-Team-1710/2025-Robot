package frc.robot.utils;

import frc.robot.subsystems.Superstructure.ReefFaces;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SimFaceChooser {
  private final LoggedDashboardChooser<ReefFaces> simFaceChooser;

  public SimFaceChooser() {
    simFaceChooser = new LoggedDashboardChooser<>("Sim Reef Face Chooser");

    simFaceChooser.addDefaultOption("ab", ReefFaces.ab);
    simFaceChooser.addOption("cd", ReefFaces.cd);
    simFaceChooser.addOption("ef", ReefFaces.ef);
    simFaceChooser.addOption("gh", ReefFaces.gh);
    simFaceChooser.addOption("ij", ReefFaces.ij);
    simFaceChooser.addOption("kl", ReefFaces.kl);
  }

  public ReefFaces getReefFace() {
    return simFaceChooser.get();
  }
}
