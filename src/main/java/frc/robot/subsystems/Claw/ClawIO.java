// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public double wristManual = 0.0;
    public double intakePercent = 0.0;

    public Angle setpoint = Degrees.of(0);

    public Angle wristMotorRotations = Rotations.of(0);
    public Angle angle = Degrees.of(0);

    public AngularVelocity wristVelocity = RotationsPerSecond.of(0);
    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0);

    public Voltage wristAppliedVoltage = Volts.of(0.0);
    public Current wristStatorCurrent = Amps.of(0);
    public Current intakeStatorCurrent = Amps.of(0);
    public Current wristSupplyCurrent = Amps.of(0);
    public Current intakeSupplyCurrent = Amps.of(0);
    public Voltage intakeAppliedVoltage = Volts.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputsAutoLogged inputs) {}

  /** Run closed loop to the specified angle. */
  public default void setAngle(Angle angle) {}

  /** Set power to the angle motor from 1 to -1 */
  public default void setManual(double power) {}

  /** Set power to the intake motor from 1 to -1 */
  public default void runPercent(double power) {}

  /**
   * Updates pid loop in ClawIOCTRE
   *
   * @param inputs
   */
  public default void updatePID(ClawIOInputsAutoLogged inputs) {}
}
