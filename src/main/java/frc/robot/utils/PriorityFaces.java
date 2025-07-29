// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Superstructure.ReefFaces;
import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class PriorityFaces {
  private final LoggedDashboardChooser<ReefFaces> reefFaceChooser;

  private ArrayList<ReefFaces> priorityReefFaces = new ArrayList<>();

  public PriorityFaces() {
    reefFaceChooser = new LoggedDashboardChooser<>("Reef Face Chooser");

    reefFaceChooser.addDefaultOption("ab", ReefFaces.ab);
    reefFaceChooser.addOption("cd", ReefFaces.cd);
    reefFaceChooser.addOption("ef", ReefFaces.ef);
    reefFaceChooser.addOption("gh", ReefFaces.gh);
    reefFaceChooser.addOption("ij", ReefFaces.ij);
    reefFaceChooser.addOption("kl", ReefFaces.kl);

    SmartDashboard.putBoolean("Lock Reef Face In", false);
    SmartDashboard.putBoolean("Remove Reef Face", false);
  }

  public void periodic() {
    if (SmartDashboard.getBoolean("Lock Reef Face In", false)
        && !priorityReefFaces.contains(reefFaceChooser.get())) {
      SmartDashboard.putBoolean("Lock Reef Face In", false);
      priorityReefFaces.add(reefFaceChooser.get());
    }
    if (SmartDashboard.getBoolean("Remove Reef Face", false)
        && priorityReefFaces.contains(reefFaceChooser.get())) {
      SmartDashboard.putBoolean("Remove Reef Face", false);
      priorityReefFaces.remove(reefFaceChooser.get());
    }
  }

  public ArrayList<ReefFaces> getPriorityFaces() {
    return priorityReefFaces;
  }
}
