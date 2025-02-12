// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.Drive.VisionParameters;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSIM extends VisionIOPhotonVision {
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  /**
   * Constructs a PhotonVision SIM object with the specified camera name and camera position.
   *
   * @param cameraName the name of the camera
   * @param robotToCamera gets positon of camera to robot
   * @param poseSupplier current pose of robot
   */
  public VisionIOPhotonVisionSIM(
      String cameraName, Transform3d robotToCamera, Supplier<VisionParameters> visionParams) {
    super(cameraName, robotToCamera, visionParams);
    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProperties.setCalibError(0.1, 0.10);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(visionParams.get().robotPose());
    super.updateInputs(inputs);
  }
}
