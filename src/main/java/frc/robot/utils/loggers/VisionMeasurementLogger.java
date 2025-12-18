package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;

@CustomLoggerFor(VisionMeasurement.class)
public class VisionMeasurementLogger extends ClassSpecificLogger<VisionMeasurement> {
  public VisionMeasurementLogger() {
    super(VisionMeasurement.class);
  }

  @Override
  public void update(EpilogueBackend backend, VisionMeasurement measurement) {
    backend.log("Ambiguity", measurement.poseEstimate().ambiguity());
    backend.log("Latency", measurement.poseEstimate().latency());
    backend.log("TagCount", measurement.poseEstimate().tagCount());
    backend.log("TimestampSeconds", measurement.poseEstimate().timestampSeconds());
    backend.log("Pose", measurement.poseEstimate().pose(), Pose3d.struct);
  }
}
