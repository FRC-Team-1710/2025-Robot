package frc.robot.utils.loggers;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers.PoseEstimate;

@CustomLoggerFor(PhotonPipelineResult.class)
public class PhotonPipelineResultLogger extends ClassSpecificLogger<PhotonPipelineResult> {
  public PhotonPipelineResultLogger() {
    super(PhotonPipelineResult.class);
  }

  @Override
  public void update(EpilogueBackend backend, PhotonPipelineResult result) {
    backend.log("Ambiguity", result.getTimestampSeconds());
    backend.log("Ambiguity", result.hasTargets());
    backend.log("Ambiguity", result.getTargets());
  }
}
