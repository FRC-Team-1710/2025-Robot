// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import frc.robot.subsystems.Superstructure.ReefFaces;
import frc.robot.subsystems.Superstructure.ReefLevel;
import frc.robot.subsystems.Superstructure.ReefSide;
import frc.robot.utils.SimCoral;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Memory {
  private final ArrayList<ReefFaces> listOfFaces;
  private final ArrayList<ReefSide> listOfSides;
  private final ArrayList<ReefLevel> listOfLevels;

  private ReefSide reefSide = ReefSide.left;
  private ReefLevel reefLevel = ReefLevel.L4;

  public Memory() {
    listOfFaces = new ArrayList<>();
    listOfSides = new ArrayList<>();
    listOfLevels = new ArrayList<>();
  }

  public void addScoredCoral(ReefFaces face, ReefSide side, ReefLevel level) {
    if (!hasScored(face, side, level)) {
      listOfFaces.add(face);
      listOfSides.add(side);
      listOfLevels.add(level);
      SimCoral.addPose(face, side, level);
    }
    ReefFaces[] newFaces = new ReefFaces[listOfFaces.size()];
    for (int i = 0; i < listOfFaces.size(); i++) {
      newFaces[i] = listOfFaces.get(i);
    }
    Logger.recordOutput("Memory/Faces", newFaces);

    ReefSide[] newSides = new ReefSide[listOfSides.size()];
    for (int i = 0; i < listOfSides.size(); i++) {
      newSides[i] = listOfSides.get(i);
    }
    Logger.recordOutput("Memory/Sides", newSides);

    ReefLevel[] newLevels = new ReefLevel[listOfLevels.size()];
    for (int i = 0; i < listOfLevels.size(); i++) {
      newLevels[i] = listOfLevels.get(i);
    }
    Logger.recordOutput("Memory/Levels", newLevels);
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

  public ReefSide getTargetSide() {
    return this.reefSide;
  }

  public ReefLevel getTargetLevel() {
    return this.reefLevel;
  }

  public ReefFaces getNextBestOption(double[] comparer, boolean isForRP) {
    ReefFaces[] facesList =
        new ReefFaces[] {
          ReefFaces.ab, ReefFaces.cd, ReefFaces.ef, ReefFaces.gh, ReefFaces.ij, ReefFaces.kl
        };

    ArrayList<ReefFaces> sortedFaces = sortBasedOnNumbers(comparer, facesList);

    return getBest(sortedFaces, isForRP);
  }

  public ArrayList<ReefFaces> sortBasedOnNumbers(double[] numberList, ReefFaces[] facesList) {
    List<Pair<Double, ReefFaces>> pairs = new ArrayList<>();
    for (int i = 0; i < numberList.length; i++) {
      pairs.add(new Pair<>(numberList[i], facesList[i]));
    }

    pairs.sort(
        new Comparator<Pair<Double, ReefFaces>>() {
          @Override
          public int compare(Pair<Double, ReefFaces> o1, Pair<Double, ReefFaces> o2) {
            return o1.getFirst().compareTo(o2.getFirst());
          }
        });

    ArrayList<ReefFaces> sortedDataArray = new ArrayList<>();

    for (int i = 0; i < pairs.size(); i++) {
      sortedDataArray.add(i, pairs.get(i).getSecond());
    }

    return sortedDataArray;
  }

  private boolean l4Done() {
    int numComplete = 0;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L4) {
        numComplete += 1;
      }
    }
    return numComplete >= 5;
  }

  private boolean l3Done() {
    int numComplete = 0;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L3) {
        numComplete += 1;
      }
    }
    return numComplete >= 5;
  }

  private boolean l2Done() {
    int numComplete = 0;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L2) {
        numComplete += 1;
      }
    }
    return numComplete >= 5;
  }

  private boolean l4Full() {
    int numComplete = 0;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L4) {
        numComplete += 1;
      }
    }
    return numComplete >= 12;
  }

  private boolean l3Full() {
    int numComplete = 0;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L3) {
        numComplete += 1;
      }
    }
    return numComplete >= 12;
  }

  private boolean l2Full() {
    int numComplete = 0;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L2) {
        numComplete += 1;
      }
    }
    return numComplete >= 12;
  }

  private boolean hasOpenL4(ReefFaces face) {
    int numComplete = 0;
    boolean leftOpen = true;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L4 && listOfFaces.get(i) == face) {
        numComplete += 1;
        if (listOfSides.get(i) == ReefSide.left) {
          leftOpen = false;
        }
      }
    }
    if (numComplete < 2) {
      reefLevel = ReefLevel.L4;
      if (leftOpen) {
        reefSide = ReefSide.left;
      } else {
        reefSide = ReefSide.right;
      }
    }
    return numComplete < 2;
  }

  private boolean hasOpenL3(ReefFaces face) {
    int numComplete = 0;
    boolean leftOpen = true;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L3 && listOfFaces.get(i) == face) {
        numComplete += 1;
        if (listOfSides.get(i) == ReefSide.left) {
          leftOpen = false;
        }
      }
    }
    if (numComplete < 2) {
      reefLevel = ReefLevel.L3;
      if (leftOpen) {
        reefSide = ReefSide.left;
      } else {
        reefSide = ReefSide.right;
      }
    }
    return numComplete < 2;
  }

  private boolean hasOpenL2(ReefFaces face) {
    int numComplete = 0;
    boolean leftOpen = true;
    for (int i = 0; i < listOfLevels.size(); i++) {
      if (listOfLevels.get(i) == ReefLevel.L2 && listOfFaces.get(i) == face) {
        numComplete += 1;
        if (listOfSides.get(i) == ReefSide.left) {
          leftOpen = false;
        }
      }
    }
    if (numComplete < 2) {
      reefLevel = ReefLevel.L2;
      if (leftOpen) {
        reefSide = ReefSide.left;
      } else {
        reefSide = ReefSide.right;
      }
    }
    return numComplete < 2;
  }

  private ReefFaces getBest(ArrayList<ReefFaces> sortedFaces, boolean isForRP) {
    if (isForRP) {
      if (l4Done()) {
        if (l3Done()) {
          if (l2Done()) {
            return getBest(sortedFaces, false);
          } else {
            while (true) {
              if (hasOpenL2(sortedFaces.get(0))) {
                return sortedFaces.get(0);
              }
              sortedFaces.remove(0);
            }
          }
        } else {
          while (true) {
            if (hasOpenL3(sortedFaces.get(0))) {
              return sortedFaces.get(0);
            }
            if (!l2Done() && hasOpenL2(sortedFaces.get(0))) {
              return sortedFaces.get(0);
            }
            sortedFaces.remove(0);
          }
        }
      } else {
        while (true) {
          if (hasOpenL4(sortedFaces.get(0))) {
            return sortedFaces.get(0);
          }
          if (!l3Done() && hasOpenL3(sortedFaces.get(0))) {
            return sortedFaces.get(0);
          }
          if (!l2Done() && hasOpenL2(sortedFaces.get(0))) {
            return sortedFaces.get(0);
          }
          sortedFaces.remove(0);
        }
      }
    } else {
      if (l4Full()) {
        if (l3Full()) {
          if (l2Full()) {
            return sortedFaces.get(0);
          } else {
            while (true) {
              if (hasOpenL2(sortedFaces.get(0))) {
                return sortedFaces.get(0);
              }
              sortedFaces.remove(0);
            }
          }
        } else {
          while (true) {
            if (hasOpenL3(sortedFaces.get(0))) {
              return sortedFaces.get(0);
            }
            if (!l2Full() && hasOpenL2(sortedFaces.get(0))) {
              return sortedFaces.get(0);
            }
            sortedFaces.remove(0);
          }
        }
      } else {
        while (true) {
          if (hasOpenL4(sortedFaces.get(0))) {
            return sortedFaces.get(0);
          }
          if (!l3Full() && hasOpenL3(sortedFaces.get(0))) {
            return sortedFaces.get(0);
          }
          if (!l2Full() && hasOpenL2(sortedFaces.get(0))) {
            return sortedFaces.get(0);
          }
          sortedFaces.remove(0);
        }
      }
    }
  }
}
