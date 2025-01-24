// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.locks.ReadWriteLock;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Alignment extends Command {
  Vision vision;
  PhotonTrackedTarget currentTarget;
  Drive drive;
  int requestedID;
  int cameraID;
  boolean atTarget;
  Transform2d offset;
  Transform2d desiredOffset;

  /** Creates a new Alignment. */
  public Alignment(Vision vision, Drive drive, int tagID, Transform2d desiredOffset) {
    this.vision = vision;
    this.drive = drive;
    this.requestedID = tagID;
    this.desiredOffset = desiredOffset;
    this.atTarget = false;
    addRequirements(vision, drive);
  }

  @Override
  public void execute() {
    for (int i = 0; i < 4; i++) {
      cameraID = i;
      if (vision.getCamera(i).hasTarget(requestedID)) {
        break;
      }
    }
    offset = VisionUtil.getTagOffset(
      vision.getCamera(cameraID).getTarget(requestedID).getBestCameraToTarget(),
      vision.getCamera(cameraID).getStdDev(),
      new Transform2d(
        new Translation2d(5.0, 0),
        new Rotation2d(0)
      ));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return atTarget;
  }
}
