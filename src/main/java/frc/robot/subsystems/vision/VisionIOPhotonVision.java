// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive.VisionParameters;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
            visionParams.get().gyroRate().in(DegreesPerSecond),
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
        System.out.println("Requested ID " + id);
        System.out.println("Actual ID " + target.fiducialId);
        if (target.fiducialId == id) {
          System.out.println("Target " + target);
          return target;
        }
      }
    }
    return null;
  }

  /**
   * Checks to see if the specified AprilTag ID is within the camera's multi-tag results
   *
   * @param id Requested AprilTag
   * @return True if the AprilTag exists in the results, false otherwise
   */
  public boolean hasTarget(int id) {
    for (var target : cameraTargets) {
      if (target.fiducialId == id) {
        return true;
      }
    }
    return false;
  }

  /**
   * Calculates the offset of the robot from a specified desired offset relative to the AprilTag
   * provided
   *
   * @param tagID Provided AprilTag ID to locate and use for calculation
   * @param desiredOffset desired position's offset relative to the AprilTag
   * @return robot centric transform 2d that represents the difference of the robots pose and the
   *     desired pose
   */
  public Translation2d getTagOffset(int tagID, Translation2d desiredOffset) {
    Translation3d robotToTargetPose;
    try {
      robotToTargetPose = getTarget(tagID).bestCameraToTarget.getTranslation().minus(robotToCamera.getTranslation());
    } catch (Exception e) {
      System.err.println("Failed to calculate offset for tag " + tagID + ": " + e.getMessage());
      return new Translation2d();
    }

    Translation2d robotOffset =
        new Translation2d(
            robotToTargetPose.getX() - desiredOffset.getX(),
            robotToTargetPose.getY() - desiredOffset.getY());
    return robotOffset;
  }
  /*
   * tagToCameraPose =
          getTarget(tagID)
              .bestCameraToTarget
              .getTranslation();
      robotToTargetPose = new Translation3d(
        -tagToCameraPose.getY() - visionStdDev.getX(),
        tagToCameraPose.getX() - visionStdDev.getY(),
        tagToCameraPose.getZ() - visionStdDev.getZ());
   */

  /**
   * Calculates the distance from a provided offset from the AprilTag to the center of the robot.
   *
   * @param tagID Provided AprilTag ID to locate and use for calculation
   * @param desiredOffset desired position's offset relative to the AprilTag
   * @return Distance to the offset from the AprilTag
   */
  public double getTagOffsetDistance(int tagID, Translation2d desiredOffset) {
    Translation3d robotToTargetPose;
    try {
      robotToTargetPose =
          getTarget(tagID)
              .bestCameraToTarget
              .getTranslation()
              .minus(robotToCamera.getTranslation());
    } catch (Exception e) {
      return 0.0;
    }
    Translation2d robotOffset =
        new Translation2d(
            robotToTargetPose.getX() - desiredOffset.getX(),
            robotToTargetPose.getY() - desiredOffset.getY());
    return robotOffset.getNorm();
  }

  public boolean hasTargets() {
    return !cameraTargets.isEmpty();
  }

  public Trigger hasTargets = new Trigger(() -> !cameraTargets.isEmpty());

  public RawFiducial result(int joystickButtonid) {
    return createRawFiducial(getTarget(joystickButtonid));
  }

  public double turn(int id) {
    double speed =
        target.getYaw()
            - getTagOffset(
                        id, new Translation2d(Units.inchesToMeters(20), Units.inchesToMeters(20)))
                    .getAngle()
                    .getDegrees()
                * 0.026553;

    return speed;
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
    PhotonPipelineResult nullResult = new PhotonPipelineResult();
    this.cameraResults = camera.getAllUnreadResults();
    this.latestResult =
        !cameraResults.isEmpty() ? cameraResults.get(cameraResults.size() - 1) : nullResult;
    this.cameraTargets = latestResult.targets;
  }
}
