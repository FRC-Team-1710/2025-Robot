// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive.VisionParameters;
import frc.robot.utils.FieldConstants;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  final PhotonCamera camera;
  private final Transform3d robotToCamera;
  final Supplier<VisionParameters> visionParams;
  List<PhotonPipelineResult> cameraResults;
  PhotonPipelineResult latestResult;
  List<PhotonTrackedTarget> cameraTargets;
  PhotonTrackedTarget target;

  boolean rejectTagsFromDistance = false;
  double tagRejectionDistance = 3.5; // METERS
  List<Integer> bargeTagIDs = List.of(4, 5, 14, 15);

  public VisionIOPhotonVision( // Creating class
      String cameraName, Transform3d robotToCamera, Supplier<VisionParameters> visionParams) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.visionParams = visionParams;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    PoseObservation observation = getEstimatedGlobalPose();
    inputs.poseEstimateMT1 = observation.poseEstimate();
    inputs.rawFiducialsMT1 = observation.rawFiducials();
  } // Auto-logs the inputs/camera measurements + info

  private PoseObservation getEstimatedGlobalPose() {
    updateResults();
    if (cameraResults.isEmpty()) return new PoseObservation();

    PhotonPipelineResult latestResult = cameraResults.get(cameraResults.size() - 1);
    if (!latestResult.hasTargets()) {
      return new PoseObservation();
    }

    if (latestResult.hasTargets()) {
      var multitagResult = latestResult.getMultiTagResult();

      if (multitagResult.isPresent()) {
        Transform3d fieldToRobot =
            multitagResult.get().estimatedPose.best.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
        return buildPoseObservation(latestResult, robotPose);
      }
      var target = latestResult.targets.get(0);
      // Calculate robot pose
      var tagPose = FieldConstants.aprilTags.getTagPose(target.fiducialId);
      if (tagPose.isPresent() && Constants.currentMode != Constants.Mode.SIM) {
        Transform3d fieldToTarget =
            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
        Transform3d cameraToTarget = target.bestCameraToTarget;
        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
        return buildPoseObservation(latestResult, robotPose);
      }
    }
    return new PoseObservation();
  }

  private PoseObservation buildPoseObservation(PhotonPipelineResult result, Pose3d robotPose) {
    List<RawFiducial> rawFiducialsList = new ArrayList<>();
    double totalDistance = 0.0;
    double totalArea = 0.0;

    for (var target : result.targets) {
      totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
      totalArea += target.area;
      rawFiducialsList.add(createRawFiducial(target));
    }

    int tagCount = result.targets.size();
    double avgDistance = tagCount > 0 ? totalDistance / tagCount : 0.0;
    double avgArea = tagCount > 0 ? totalArea / tagCount : 0.0;
    double ambiguity = tagCount > 0 ? rawFiducialsList.get(0).ambiguity() : 0.0;

    return new PoseObservation(
        new PoseEstimate(
            robotPose,
            result.getTimestampSeconds(),
            0.0,
            tagCount,
            0.0,
            avgDistance,
            avgArea,
            ambiguity,
            visionParams.get().gyroRate(),
            visionParams.get().robotPose(),
            false),
        rawFiducialsList.toArray(new RawFiducial[0]));
  }

  /**
   * Provides the offset of the camera relative to the robot
   *
   * @return Standard deviations from the robot to the camera
   */
  public Transform3d getStdDev() {
    return robotToCamera;
  }

  /**
   * Gets the least ambiguous AprilTag in the multi-tag results
   *
   * @return Least ambiguous AprilTag in camera's view
   */
  public PhotonTrackedTarget getBestTarget() {
    return latestResult.getBestTarget();
  }

  /**
   * Gets the specified AprilTag from the multi-tag results
   *
   * @return Specified AprilTag or null if it isn't in the camera's view
   */
  public PhotonTrackedTarget getTarget(int id) {
    if (!cameraTargets.isEmpty()) {
      for (var target : cameraTargets) {
        if (target.fiducialId == id) {
          return target;
        }
      }
    }
    return new PhotonTrackedTarget();
  }

  /**
   * Checks to see if the specified AprilTag ID is within the camera's multi-tag results
   *
   * @param id Requested AprilTag
   * @return True if the AprilTag exists in the results, false otherwise
   */
  public boolean hasTarget(int id) {
    if (!cameraTargets.isEmpty()) {
      for (var target : cameraTargets) {
        if (target.fiducialId == id) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Calculates the offset of the robot from a specified desired offset relative to the AprilTag
   * provided
   *
   * @param tagID Provided AprilTag ID to locate and use for calculation
   */
  public Transform3d getRobotToTargetOffset(int tagID) {
    Transform3d tagToCameraPose;
    try {
      tagToCameraPose = getTarget(tagID).bestCameraToTarget.inverse().plus(robotToCamera.inverse());
    } catch (Exception e) {
      return new Transform3d();
    }
    Logger.recordOutput("VisionDebugging/tagToCameraPose via " + camera.getName(), tagToCameraPose);
    return tagToCameraPose;
  }

  /**
   * Provides the Transform3d that represents the robots position relative to the provided AprilTag
   *
   * @param id Requested tag ID
   * @return Returns the robots position relative to the AprilTag
   */
  @AutoLogOutput
  public Transform3d getTransformToTag(int id) {
    if (latestResult.hasTargets()) {
      for (var target : latestResult.getTargets()) {
        if (target.fiducialId == id) {
          return target.bestCameraToTarget.plus(robotToCamera);
        }
      }
    }
    return new Transform3d(new Translation3d(3, 0, 0), new Rotation3d());
  }

  /**
   * Checks to make sure that the camera has AprilTag results/targets
   *
   * @return Boolean to represent the presence of AprilTag results
   */
  public boolean hasTargets() {
    return !cameraTargets.isEmpty();
  }

  /** Redundant perchance. */
  public Trigger hasTargets = new Trigger(() -> !cameraTargets.isEmpty());

  /** Also probably redundant perchance. */
  public RawFiducial result(int joystickButtonid) {
    return createRawFiducial(getTarget(joystickButtonid));
  }

  /**
   * Provides the camera's most recent targets
   *
   * @return List of targets
   */
  public List<PhotonTrackedTarget> getCameraTargets() {
    return cameraTargets;
  }

  /**
   * Enables or disables rejecting tags from a distance
   *
   * @param useRejectionDistance Boolean to set whether the camera can reject tags from a distance
   *     or not
   */
  public void useRejectionDistance(boolean useRejectionDistance) {
    this.rejectTagsFromDistance = useRejectionDistance;
  }

  /**
   * Sets the camera's rejection distance and allows the camera to reject the tags further than this
   * distance.4
   *
   * @param rejectionDistance Preferred camera range in meters
   */
  public void useRejectionDistance(double rejectionDistance) {
    this.tagRejectionDistance = rejectionDistance;
    this.rejectTagsFromDistance = true;
  }

  private RawFiducial createRawFiducial(PhotonTrackedTarget target) {
    return new RawFiducial(
        target.getFiducialId(),
        0,
        0,
        target.area,
        target.bestCameraToTarget.getTranslation().minus(robotToCamera.getTranslation()).getNorm(),
        target.bestCameraToTarget.getTranslation().getNorm(),
        target.poseAmbiguity);
  }

  /** Updates the local vision results variables */
  private void updateResults() {
    cameraResults = camera.getAllUnreadResults();
    if (!cameraResults.isEmpty()) {
      latestResult = cameraResults.get(cameraResults.size() - 1);
    } else {
      latestResult = new PhotonPipelineResult();
    }
    if (latestResult.hasTargets()) {
      cameraTargets = latestResult.targets;
    } else {
      cameraTargets = new ArrayList<>();
    }

    // Filtering //
    if (latestResult.hasTargets()) {
      for (int tagIndex = 0; tagIndex < cameraTargets.size(); tagIndex++) {
        if (bargeTagIDs.contains(cameraTargets.get(tagIndex).fiducialId)) {
          removeTag(tagIndex);
        }
      }

      if (rejectTagsFromDistance) {
        List<PhotonTrackedTarget> tags = latestResult.targets;
        for (int tagIndex = 0; tagIndex < tags.size(); tagIndex++) {
          if (tags.get(tagIndex).bestCameraToTarget.getTranslation().getNorm()
              > tagRejectionDistance) {
            removeTag(tagIndex);
          }
        }
      }
    }
  }

  private void removeTag(int index) {
    cameraResults.remove(index);
    cameraTargets.remove(index);
  }
}
