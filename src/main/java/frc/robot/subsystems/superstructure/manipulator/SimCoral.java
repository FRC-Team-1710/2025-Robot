// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TargetingComputer.Levels;
import frc.robot.utils.TargetingComputer.Targets;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SimCoral {
  private static ArrayList<Pose3d> poses = new ArrayList<>();

  public static void start() {
    poses.add(new Pose3d());
  }

  public static void setPose(int index, Pose3d pose) {
    poses.set(index, pose);
    Logger.recordOutput("Coral", poses.toArray(new Pose3d[poses.size()]));
  }

  public static void placeCoral() {
    Pose2d pose = new Pose2d();
    spots spot =
        getSpot(
            TargetingComputer.getCurrentTargetBranch(), TargetingComputer.getCurrentTargetLevel());
    if (spot != null && spot != spots.Level1) {
      poses.add(
          getSpot(
                  TargetingComputer.getCurrentTargetBranch(),
                  TargetingComputer.getCurrentTargetLevel())
              .pose);
    } else if (spot == spots.Level1) {
      if (pose.getTranslation().getDistance(Constants.CenterOfReef) <= Units.inchesToMeters(50)) {
        poses.add(getAngledPose(pose));
      }
    }
    poses.add(
        getSpot(
                    TargetingComputer.getCurrentTargetBranch(),
                    TargetingComputer.getCurrentTargetLevel())
                != null
            ? getSpot(
                    TargetingComputer.getCurrentTargetBranch(),
                    TargetingComputer.getCurrentTargetLevel())
                .pose
            : new Pose3d());
    Logger.recordOutput("Coral", poses.toArray(new Pose3d[poses.size()]));
  }

  public static void placeCoral(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rad) {
    Pose2d pose = new Pose2d(x.getAsDouble(), y.getAsDouble(), new Rotation2d(rad.getAsDouble()));
    spots spot = spots.Level1;
    if (spot != null && spot != spots.Level1) {
      poses.add(
          getSpot(
                  TargetingComputer.getCurrentTargetBranch(),
                  TargetingComputer.getCurrentTargetLevel())
              .pose);
    } else if (spot == spots.Level1) {
      if (pose.getTranslation().getDistance(Constants.CenterOfReef) <= Units.inchesToMeters(60)) {
        poses.add(getAngledPose(pose));
      }
    }
    Logger.recordOutput("Coral", poses.toArray(new Pose3d[poses.size()]));
  }

  public static Pose3d getAngledPose(Pose2d pose) {
    pose =
        new Pose2d(
            pose.getX(),
            pose.getY(),
            new Rotation2d(Units.degreesToRadians(-pose.getRotation().getDegrees())));
    double posex = pose.getX();
    double posey = pose.getY();
    double placeDistance = 0.4;

    if (pose.getRotation().getDegrees() > 0 && pose.getRotation().getDegrees() < 90) {
      posey =
          pose.getY()
              + (Math.cos(pose.getRotation().getRadians()) * placeDistance); // Top right of circle
    } else if (pose.getRotation().getDegrees() < -90 && pose.getRotation().getDegrees() > -180) {
      posey =
          pose.getY()
              + (Math.cos(Units.degreesToRadians(pose.getRotation().getDegrees() + 90))
                  * placeDistance); // Bottom right of circle
    } else if (pose.getRotation().getDegrees() > 0 && pose.getRotation().getDegrees() < -90) {
      posey =
          pose.getY()
              - (Math.cos(pose.getRotation().getRadians()) * placeDistance); // Top left of circle
    } else if (pose.getRotation().getDegrees() > 90 && pose.getRotation().getDegrees() < -180) {
      posey =
          pose.getY()
              - (Math.cos(Units.degreesToRadians(pose.getRotation().getDegrees() - 90))
                  * placeDistance); // Bottom left of circle
    }

    if (pose.getRotation().getDegrees() > 0 && pose.getRotation().getDegrees() < 90) {
      posex =
          pose.getX()
              - (Math.sin(pose.getRotation().getRadians()) * placeDistance); // Top right of circle
    } else if (pose.getRotation().getDegrees() < -90 && pose.getRotation().getDegrees() > -180) {
      posex =
          pose.getX()
              + (Math.sin(Units.degreesToRadians(pose.getRotation().getDegrees() + 90))
                  * placeDistance); // Bottom right of circle
    } else if (pose.getRotation().getDegrees() > 0 && pose.getRotation().getDegrees() < -90) {
      posex =
          pose.getX()
              - (Math.sin(pose.getRotation().getRadians()) * placeDistance); // Top left of circle
    } else if (pose.getRotation().getDegrees() > 90 && pose.getRotation().getDegrees() < -180) {
      posex =
          pose.getX()
              + (Math.sin(Units.degreesToRadians(pose.getRotation().getDegrees() - 90))
                  * placeDistance); // Bottom left of circle
    }

    return new Pose3d(
        posex,
        posey,
        0.6,
        new Rotation3d(0, Units.degreesToRadians(-80), pose.getRotation().getRadians()));
  }

  public static void tempCoral() {
    Logger.recordOutput(
        "Coral",
        new Pose3d(
            SmartDashboard.getNumber("Coral/x", 0),
            SmartDashboard.getNumber("Coral/y", 0),
            SmartDashboard.getNumber("Coral/z", 0),
            new Rotation3d(
                Units.degreesToRadians(SmartDashboard.getNumber("Coral/roll", 0)),
                Units.degreesToRadians(SmartDashboard.getNumber("Coral/pitch", 0)),
                Units.degreesToRadians(SmartDashboard.getNumber("Coral/yaw", 0)))));
  }

  public static Command intake(
      DoubleSupplier posex, DoubleSupplier posey, DoubleSupplier poserotation) {
    return new Intake(posex, posey, poserotation);
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

  public static spots getSpot(Targets target, Levels level) {
    if (target == Targets.ALPHA) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.A2;
      }
      if (level == Levels.L3) {
        return spots.A3;
      }
      if (level == Levels.L4) {
        return spots.A4;
      }
    }
    if (target == Targets.BRAVO) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.B2;
      }
      if (level == Levels.L3) {
        return spots.B3;
      }
      if (level == Levels.L4) {
        return spots.B4;
      }
    }
    if (target == Targets.CHARLIE) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.C2;
      }
      if (level == Levels.L3) {
        return spots.C3;
      }
      if (level == Levels.L4) {
        return spots.C4;
      }
    }
    if (target == Targets.DELTA) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.D2;
      }
      if (level == Levels.L3) {
        return spots.D3;
      }
      if (level == Levels.L4) {
        return spots.D4;
      }
    }
    if (target == Targets.ECHO) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.E2;
      }
      if (level == Levels.L3) {
        return spots.E3;
      }
      if (level == Levels.L4) {
        return spots.E4;
      }
    }
    if (target == Targets.FOXTROT) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.F2;
      }
      if (level == Levels.L3) {
        return spots.F3;
      }
      if (level == Levels.L4) {
        return spots.F4;
      }
    }
    if (target == Targets.GOLF) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.G2;
      }
      if (level == Levels.L3) {
        return spots.G3;
      }
      if (level == Levels.L4) {
        return spots.G4;
      }
    }
    if (target == Targets.HOTEL) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.H2;
      }
      if (level == Levels.L3) {
        return spots.H3;
      }
      if (level == Levels.L4) {
        return spots.H4;
      }
    }
    if (target == Targets.INDIA) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.I2;
      }
      if (level == Levels.L3) {
        return spots.I3;
      }
      if (level == Levels.L4) {
        return spots.I4;
      }
    }
    if (target == Targets.JULIET) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.J2;
      }
      if (level == Levels.L3) {
        return spots.J3;
      }
      if (level == Levels.L4) {
        return spots.J4;
      }
    }
    if (target == Targets.KILO) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.K2;
      }
      if (level == Levels.L3) {
        return spots.K3;
      }
      if (level == Levels.L4) {
        return spots.K4;
      }
    }
    if (target == Targets.LIMA) {
      if (level == Levels.L1) {
        return spots.Level1;
      }
      if (level == Levels.L2) {
        return spots.L2;
      }
      if (level == Levels.L3) {
        return spots.L3;
      }
      if (level == Levels.L4) {
        return spots.L4;
      }
    }
    return null;
  }
}
