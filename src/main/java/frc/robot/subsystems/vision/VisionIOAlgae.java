package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class VisionIOAlgae {
  @NotLogged final PhotonCamera camera;
  @NotLogged List<PhotonPipelineResult> cameraResults;

  @Logged(name = "CurrentResult", importance = Importance.CRITICAL)
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

  @Logged(name = "TargetVisible", importance = Importance.CRITICAL)
  public boolean targetVisible() {
    return Constants.currentMode == Mode.REAL
        ? ((!cameraResults.isEmpty() || !(cameraResults == null)) && currentResult.hasTargets())
        : false;
  }

  // LEFT BUMPER
  @Logged(name = "AlgaeYaw", importance = Importance.CRITICAL)
  public double getAlgaeYaw() {
    if (Constants.currentMode == Mode.REAL) {
      boolean targetVisible = false;
      double targetYaw = 0.0;
      double lowestPitch = 180.0;
      if ((!cameraResults.isEmpty() || !(cameraResults == null)) && currentResult != null) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        Boolean hasResults = currentResult.hasTargets();
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
      return targetVisible ? targetYaw : 0.0;
    }
    return 0;
  }
}
