package frc.robot.utils.loggers;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX talon) {
    backend.log("Id", talon.getDeviceID());
    backend.log("ClosedLoopError", talon.getClosedLoopError().getValueAsDouble());
    backend.log("IsProLicensed", talon.getIsProLicensed().getValueAsDouble());
    backend.log("MotorVoltage", talon.getMotorVoltage().getValueAsDouble());
    backend.log("Velocity", talon.getVelocity().getValueAsDouble());
    backend.log("StatorCurrent", talon.getStatorCurrent().getValueAsDouble());
    backend.log("Position", talon.getPosition().getValueAsDouble());
  }
}
