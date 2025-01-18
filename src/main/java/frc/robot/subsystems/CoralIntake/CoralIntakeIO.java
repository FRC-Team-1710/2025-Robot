package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public class CoralIntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean beam1Broken = false;
    public boolean beam2Broken = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
