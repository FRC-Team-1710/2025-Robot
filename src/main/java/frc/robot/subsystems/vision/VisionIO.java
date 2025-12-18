// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

@Logged
public interface VisionIO {
  @Logged
  public static class VisionIOInputs {
    @Logged(name = "Connected", importance = Importance.CRITICAL)
    boolean connected = false;

    @Logged(name = "PoseEstimateMT1", importance = Importance.CRITICAL)
    PoseEstimate poseEstimateMT1 = new PoseEstimate();

    @Logged(name = "PoseEstimateMT2", importance = Importance.CRITICAL)
    PoseEstimate poseEstimateMT2 = new PoseEstimate();

    @NotLogged RawFiducial[] rawFiducialsMT1 = new RawFiducial[0];
    @NotLogged RawFiducial[] rawFiducialsMT2 = new RawFiducial[0];
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
