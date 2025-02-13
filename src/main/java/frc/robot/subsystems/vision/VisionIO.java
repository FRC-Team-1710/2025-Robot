// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    boolean connected = false;
    PoseEstimate poseEstimateMT1 = new PoseEstimate();
    PoseEstimate poseEstimateMT2 = new PoseEstimate();
    RawFiducial[] rawFiducialsMT1 = new RawFiducial[0];
    RawFiducial[] rawFiducialsMT2 = new RawFiducial[0];
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
