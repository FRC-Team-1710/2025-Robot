package frc.robot.utils.loggers;

import static edu.wpi.first.units.Units.Inches;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

@CustomLoggerFor(APTarget.class)
public class APTargetLogger extends ClassSpecificLogger<APTarget> {
  public APTargetLogger() {
    super(APTarget.class);
  }

  @Override
  public void update(EpilogueBackend backend, APTarget target) {
    backend.log("Velocity", target.getVelocity());
    backend.log("EntryAngle", target.getEntryAngle().isEmpty() ? Rotation2d.kZero : target.getEntryAngle().get(), Rotation2d.struct);
    backend.log("Reference", target.getReference(), Pose2d.struct);
    backend.log("RotationRadius", target.getRotationRadius().isEmpty() ? Inches.of(0) : target.getRotationRadius().get());
  }
}
