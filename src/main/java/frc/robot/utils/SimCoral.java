// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Superstructure.ReefFaces;
import frc.robot.subsystems.Superstructure.ReefLevel;
import frc.robot.subsystems.Superstructure.ReefSide;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class SimCoral {
  private static ArrayList<Pose3d> poses = new ArrayList<>();

  private static boolean isRedAlliance = false;

  public static void setRedAlliance(boolean redAlliance) {
    isRedAlliance = redAlliance;
  }

  public static void addPose(ReefFaces face, ReefSide side, ReefLevel level) {
    poses.add(
        apply(
            switch (face) {
              case ab ->
                  switch (level) {
                    case L2 ->
                        switch (side) {
                          case left -> spots.A2.pose;
                          case right -> spots.B2.pose;
                        };
                    case L3 ->
                        switch (side) {
                          case left -> spots.A3.pose;
                          case right -> spots.B3.pose;
                        };
                    case L4 ->
                        switch (side) {
                          case left -> spots.A4.pose;
                          case right -> spots.B4.pose;
                        };
                    default -> spots.Level1.pose;
                  };
              case cd ->
                  switch (level) {
                    case L2 ->
                        switch (side) {
                          case left -> spots.C2.pose;
                          case right -> spots.D2.pose;
                        };
                    case L3 ->
                        switch (side) {
                          case left -> spots.C3.pose;
                          case right -> spots.D3.pose;
                        };
                    case L4 ->
                        switch (side) {
                          case left -> spots.C4.pose;
                          case right -> spots.D4.pose;
                        };
                    default -> spots.Level1.pose;
                  };
              case ef ->
                  switch (level) {
                    case L2 ->
                        switch (side) {
                          case left -> spots.E2.pose;
                          case right -> spots.F2.pose;
                        };
                    case L3 ->
                        switch (side) {
                          case left -> spots.E3.pose;
                          case right -> spots.F3.pose;
                        };
                    case L4 ->
                        switch (side) {
                          case left -> spots.E4.pose;
                          case right -> spots.F4.pose;
                        };
                    default -> spots.Level1.pose;
                  };
              case gh ->
                  switch (level) {
                    case L2 ->
                        switch (side) {
                          case left -> spots.G2.pose;
                          case right -> spots.H2.pose;
                        };
                    case L3 ->
                        switch (side) {
                          case left -> spots.G3.pose;
                          case right -> spots.H3.pose;
                        };
                    case L4 ->
                        switch (side) {
                          case left -> spots.G4.pose;
                          case right -> spots.H4.pose;
                        };
                    default -> spots.Level1.pose;
                  };
              case ij ->
                  switch (level) {
                    case L2 ->
                        switch (side) {
                          case left -> spots.I2.pose;
                          case right -> spots.J2.pose;
                        };
                    case L3 ->
                        switch (side) {
                          case left -> spots.I3.pose;
                          case right -> spots.J3.pose;
                        };
                    case L4 ->
                        switch (side) {
                          case left -> spots.I4.pose;
                          case right -> spots.J4.pose;
                        };
                    default -> spots.Level1.pose;
                  };
              case kl ->
                  switch (level) {
                    case L2 ->
                        switch (side) {
                          case left -> spots.K2.pose;
                          case right -> spots.L2.pose;
                        };
                    case L3 ->
                        switch (side) {
                          case left -> spots.K3.pose;
                          case right -> spots.L3.pose;
                        };
                    case L4 ->
                        switch (side) {
                          case left -> spots.K4.pose;
                          case right -> spots.L4.pose;
                        };
                    default -> spots.Level1.pose;
                  };
            }));
    Pose3d[] newPoses = new Pose3d[poses.size()];
    for (int i = 0; i < poses.size(); i++) {
      newPoses[i] = poses.get(i);
    }
    Logger.recordOutput("SimCoralPoses", newPoses);
  }

  public static enum spots {
    Level1(),
    A2(
        new Pose3d(
            3.78,
            4.189,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(0.0)))),
    B2(
        new Pose3d(
            3.78,
            3.8625,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(0.0)))),
    C2(
        new Pose3d(
            3.99,
            3.49,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(60.0)))),
    D2(
        new Pose3d(
            4.275,
            3.325,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(60.0)))),
    E2(
        new Pose3d(
            4.705,
            3.325,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(120.0)))),
    F2(
        new Pose3d(
            4.99,
            3.49,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(120.0)))),
    G2(
        new Pose3d(
            5.2,
            3.8625,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(180.0)))),
    H2(
        new Pose3d(
            5.2,
            4.189,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(180.0)))),
    I2(
        new Pose3d(
            4.99,
            4.56,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(240.0)))),
    J2(
        new Pose3d(
            4.705,
            4.73,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(240.0)))),
    K2(
        new Pose3d(
            4.275,
            4.73,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(300.0)))),
    L2(
        new Pose3d(
            3.99,
            4.56,
            0.725,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(300.0)))),
    A3(
        new Pose3d(
            3.78,
            4.189,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(0.0)))),
    B3(
        new Pose3d(
            3.78,
            3.8625,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(0.0)))),
    C3(
        new Pose3d(
            3.99,
            3.49,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(60.0)))),
    D3(
        new Pose3d(
            4.275,
            3.325,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(60.0)))),
    E3(
        new Pose3d(
            4.705,
            3.325,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(120.0)))),
    F3(
        new Pose3d(
            4.99,
            3.49,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(120.0)))),
    G3(
        new Pose3d(
            5.2,
            3.8625,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(180.0)))),
    H3(
        new Pose3d(
            5.2,
            4.189,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(180.0)))),
    I3(
        new Pose3d(
            4.99,
            4.56,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(240.0)))),
    J3(
        new Pose3d(
            4.705,
            4.73,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(240.0)))),
    K3(
        new Pose3d(
            4.275,
            4.73,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(300.0)))),
    L3(
        new Pose3d(
            3.99,
            4.56,
            1.125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(30.0),
                Units.degreesToRadians(300.0)))),
    A4(
        new Pose3d(
            3.71,
            4.198,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(0.0)))),
    B4(
        new Pose3d(
            3.71,
            3.8625,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(0.0)))),
    C4(
        new Pose3d(
            3.96,
            3.435,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(60.0)))),
    D4(
        new Pose3d(
            4.245,
            3.2775,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(60.0)))),
    E4(
        new Pose3d(
            4.725,
            3.2775,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(120.0)))),
    F4(
        new Pose3d(
            5.022,
            3.435,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(120.0)))),
    G4(
        new Pose3d(
            5.265,
            3.8625,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(180.0)))),
    H4(
        new Pose3d(
            5.265,
            4.189,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(180.0)))),
    I4(
        new Pose3d(
            5.022,
            4.625,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(240.0)))),
    J4(
        new Pose3d(
            4.7325,
            4.785,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(240.0)))),
    K4(
        new Pose3d(
            4.245,
            4.785,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(300.0)))),
    L4(
        new Pose3d(
            3.96,
            4.625,
            1.7125,
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(75.0),
                Units.degreesToRadians(300.0))));

    private final Pose3d pose;

    spots(Pose3d pose) {
      this.pose = pose;
    }

    spots() {
      this.pose = new Pose3d();
    }
  }

  public static Pose3d apply(Pose3d pose) {
    return isRedAlliance
        ? new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getMeasureX()),
        applyY(translation.getMeasureY()),
        translation.getMeasureZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return isRedAlliance
        ? rotation.rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(180)))
        : rotation;
  }

  public static Distance applyX(Distance x) {
    return isRedAlliance ? FieldConstants.fieldLength.minus(x) : x;
  }

  public static Distance applyY(Distance y) {
    return isRedAlliance ? FieldConstants.fieldWidth.minus(y) : y;
  }
}
