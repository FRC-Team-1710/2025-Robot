// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.Superstructure.ReefFaces;
import frc.robot.subsystems.Superstructure.ReefLevel;
import frc.robot.subsystems.Superstructure.ReefSide;
import frc.robot.utils.SimCoral;
import java.util.ArrayList;

public class Memory {
  private final ArrayList<ReefFaces> listOfFaces;
  private final ArrayList<ReefSide> listOfSides;
  private final ArrayList<ReefLevel> listOfLevels;

  public Memory() {
    listOfFaces = new ArrayList<>();
    listOfSides = new ArrayList<>();
    listOfLevels = new ArrayList<>();
  }

  public void addScoredCoral(ReefFaces face, ReefSide side, ReefLevel level) {
    listOfFaces.add(face);
    listOfSides.add(side);
    listOfLevels.add(level);
    SimCoral.addPose(face, side, level);
  }

  public boolean hasScored(ReefFaces face, ReefSide side, ReefLevel level) {
    for (int i = 0; i < listOfFaces.size(); i++) {
      if (face == listOfFaces.get(i)
          && side == listOfSides.get(i)
          && level == listOfLevels.get(i)) {
        return true;
      }
    }
    return false;
  }
}
