package frc.robot.subsystems.vision;

import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOAlgae {
  final PhotonCamera camera;
  List<PhotonPipelineResult> cameraResults;
  PhotonPipelineResult currentResult;

  public VisionIOAlgae( // Creating class
      String cameraName) {
    this.camera = new PhotonCamera(cameraName);
  }

  public void updateResults() {
    this.cameraResults = camera.getAllUnreadResults();
    if (cameraResults == null || !cameraResults.isEmpty()) {
      this.currentResult = cameraResults.get(cameraResults.size() - 1);
    }
  }

  public boolean targetVisible() {
    return (!cameraResults.isEmpty() || !(cameraResults == null)) && currentResult.hasTargets();
  }

  // LEFT BUMBPER
  public double getAlgaeYaw() {
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double lowestPitch = 180.0;
    if (!cameraResults.isEmpty() || !(cameraResults == null)) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      Boolean hasResults = currentResult.hasTargets();
      Logger.recordOutput("Has Targets", hasResults);
      if (hasResults) {
        // At least one AprilTag was seen by the camera
        for (var target : currentResult.getTargets()) {
          double pitch = target.getPitch();
          if (pitch < lowestPitch) {
            targetYaw = target.getYaw();
            lowestPitch = pitch;
          }
          targetVisible = true;
        }
      }
    }
    Logger.recordOutput("Lowest Pitch", lowestPitch);
    return targetVisible ? targetYaw : 0.0;
  }
}
