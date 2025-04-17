package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOAlgae {
  final PhotonCamera camera;
  List<PhotonPipelineResult> cameraResults;

  public VisionIOAlgae( // Creating class
      String cameraName) {
    this.camera = new PhotonCamera(cameraName);
  }

  public void updateResults() {
    this.cameraResults = camera.getAllUnreadResults();
  }

  // LEFT BUMBPER
  public double getAlgaeYaw() {
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double lowestPitch = 0.0;
    if (!cameraResults.isEmpty() || !(cameraResults == null)) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = cameraResults.get(cameraResults.size() - 1);
      Logger.recordOutput("Has Targets", result.hasTargets());
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getPitch() < lowestPitch || lowestPitch == 0.0) {
            targetYaw = target.getYaw();
            lowestPitch = target.getPitch();
          }
          targetVisible = true;
        }
      }
    }
    return targetVisible ? targetYaw : 0.0;
  }
}
