package frc.robot.utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.AutoLogOutput;

public class LoggableTalonFX extends TalonFX {
  private final String subsystemName;
  private final Alert motorDisconnectedAlert;

  public LoggableTalonFX(int deviceId, String subsystemName) {
    super(deviceId, "");
    this.subsystemName = subsystemName;
    motorDisconnectedAlert =
        new Alert(
            "Disconnected Motor ID: "
                + Integer.toString(deviceId)
                + " Subsystem: "
                + subsystemName
                + ".",
            AlertType.kError);
    this.optimizeBusUtilization(4, 0.1);
  }

  public LoggableTalonFX(int deviceId, CANBus canbus, String subsystemName) {
    super(deviceId, canbus.getName());
    this.subsystemName = subsystemName;
    motorDisconnectedAlert =
        new Alert(
            "Disconnected Motor ID: "
                + Integer.toString(deviceId)
                + "CANBus: "
                + canbus.getName()
                + " Subsystem: "
                + subsystemName
                + ".",
            AlertType.kError);
    this.optimizeBusUtilization(4, 0.1);
  }

  @AutoLogOutput(key = "{subsystemName}/Position")
  private Angle logPosition() {
    return super.getPosition().getValue();
  }

  @AutoLogOutput(key = "{subsystemName}/Velocity")
  private AngularVelocity logVelocity() {
    return super.getVelocity().getValue();
  }

  @AutoLogOutput(key = "{subsystemName}/MotorVoltage")
  private Voltage logMotorVoltage() {
    return super.getMotorVoltage().getValue();
  }

  @AutoLogOutput(key = "{subsystemName}/StatorCurrent")
  private Current logStatorCurrent() {
    return super.getStatorCurrent().getValue();
  }

  @AutoLogOutput(key = "{subsystemName}/SupplyCurrent")
  private Current logSupplyCurrent() {
    return super.getSupplyCurrent().getValue();
  }

  @AutoLogOutput(key = "{subsystemName}/DeviceTemp")
  private Temperature logDeviceTemp() {
    return super.getDeviceTemp().getValue();
  }

  @AutoLogOutput(key = "{subsystemName}/Connected")
  private boolean logConnected() {
    motorDisconnectedAlert.set(!super.isConnected());
    return super.isConnected();
  }

  @AutoLogOutput(key = "{subsystemName}/AppliedControl")
  private String logAppliedControl() {
    return super.getAppliedControl().getName();
  }

  @AutoLogOutput(key = "{subsystemName}/ControlInfo")
  private String logControlInfo() {
    StringBuilder sb = new StringBuilder();
    super.getAppliedControl()
        .getControlInfo()
        .forEach((key, value) -> sb.append(key).append(" = ").append(value).append("\n"));
    return sb.toString();
  }

  public String getSubsystemName() {
    return subsystemName;
  }
}
