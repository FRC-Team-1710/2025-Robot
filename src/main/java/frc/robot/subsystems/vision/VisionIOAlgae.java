package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
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
    if (cameraResults != null && !cameraResults.isEmpty()) {
      this.currentResult = cameraResults.get(cameraResults.size() - 1);
    } else {
      this.currentResult = null; // necessary for sim
    }
  }

  public boolean targetVisible() {
    return Constants.currentMode == Mode.REAL
        ? ((cameraResults != null && !cameraResults.isEmpty()) && currentResult.hasTargets())
        : false;
  }

  // LEFT BUMBPER
  public double getAlgaeYaw() {
    if (Constants.currentMode == Mode.REAL) {
      // Default to not visible,
      double targetYaw = Double.NaN;
      double lowestPitch = Double.POSITIVE_INFINITY;
      if ((cameraResults != null && !cameraResults.isEmpty()) && currentResult != null) {
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
            // Finds the target with the lowest pitch (closest to robot)
          }
        }
      }
      Logger.recordOutput("Lowest Pitch", lowestPitch);
      return targetYaw;
    }
    return Double.NaN;
  }
}
