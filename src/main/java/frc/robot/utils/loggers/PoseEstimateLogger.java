package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers.PoseEstimate;

@CustomLoggerFor(PoseEstimate.class)
public class PoseEstimateLogger extends ClassSpecificLogger<PoseEstimate> {
  public PoseEstimateLogger() {
    super(PoseEstimate.class);
  }

  @Override
  public void update(EpilogueBackend backend, PoseEstimate poseEstimate) {
    backend.log("Ambiguity", poseEstimate.ambiguity());
    backend.log("Latency", poseEstimate.latency());
    backend.log("TagCount", poseEstimate.tagCount());
    backend.log("TimestampSeconds", poseEstimate.timestampSeconds());
    backend.log("Pose", poseEstimate.pose(), Pose3d.struct);
  }
}
