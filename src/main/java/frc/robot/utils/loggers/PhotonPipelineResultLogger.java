package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.photonvision.targeting.PhotonPipelineResult;

@CustomLoggerFor(PhotonPipelineResult.class)
public class PhotonPipelineResultLogger extends ClassSpecificLogger<PhotonPipelineResult> {
  public PhotonPipelineResultLogger() {
    super(PhotonPipelineResult.class);
  }

  @Override
  public void update(EpilogueBackend backend, PhotonPipelineResult result) {
    // backend.log("Ambiguity", result.getTimestampSeconds());
    // backend.log("Ambiguity", result.hasTargets());
    // backend.log("Ambiguity", result.getTargets());
  }
}
