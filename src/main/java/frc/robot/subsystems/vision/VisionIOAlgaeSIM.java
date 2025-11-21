package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionIOAlgaeSIM extends VisionIOAlgae {

  private static final double ALGAE_DIAMETER_METERS = 0.4064;

  private final VisionSystemSim simSystem;
  private final PhotonCameraSim simCamera;
  private final Supplier<Transform3d> robotPoseSupplier;

  public VisionIOAlgaeSIM(
      String cameraName,
      Transform3d robotToCamera,
      Supplier<Transform3d> robotPoseSupplier) {

    super(cameraName);

    this.robotPoseSupplier = robotPoseSupplier;

    // Create the sim system once
    simSystem = new VisionSystemSim("algaeSim");

    // Camera properties: FOV, resolution, etc.
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    props.setCalibError(0, 0);

    // Add camera
    simCamera = new PhotonCameraSim(camera, props);
    simSystem.addCamera(simCamera, robotToCamera);
    simCamera.enableDrawWireframe(true);
  }

  // Call this every sim tick
  @Override
  public void updateResults() {
    simSystem.update(robotPoseSupplier.get());
    super.updateResults(); // Call your real camera-to-results logic
  }

  // Feed sim algae models
  public void updateAlgaeModels(VisionTargetSim[] algaeTargets) {
    simSystem.clearVisionTargets();
    for (int i = 0; i < algaeTargets.length; i++) {
      simSystem.addVisionTargets("Algae" + i, algaeTargets[i]);
    }
  }

  // Helper to create a target for an algae ball
  public static VisionTargetSim makeAlgaeTarget(
      Transform3d fieldToTarget) {

    return new VisionTargetSim(
        fieldToTarget,
        ALGAE_DIAMETER_METERS,
        ALGAE_DIAMETER_METERS);
  }
}
