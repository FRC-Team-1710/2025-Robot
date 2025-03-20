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

package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
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

    public boolean clawConnected = false;
    public boolean wristConnected = false;

    public boolean hasAlgae = false;
    public boolean rollerLocked = false;

    public boolean killSwich = false;

    public Angle wristMotorAngle = Degrees.of(0);
    public Angle angle = Degrees.of(0);

    public AngularVelocity wristVelocity = DegreesPerSecond.of(0);
    public AngularVelocity intakeVelocity = DegreesPerSecond.of(0);

    public Voltage wristAppliedVoltage = Volts.of(0.0);
    public Current wristStatorCurrent = Amps.of(0);
    public Current wristSupplyCurrent = Amps.of(0);
    public Current rollerStatorCurrent = Amps.of(0);
    public Current rollerSupplyCurrent = Amps.of(0);
    public Voltage rollerAppliedVoltage = Volts.of(0.0);
    public double rollerPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  /** Run closed loop to the specified angle. */
  public default void setAngle(Angle angle) {}

  /** Set power to the angle motor from 1 to -1 */
  public default void wristManual(double power) {}

  /** Set power to the intake motor from 1 to -1 */
  public default void setRollers(double power) {}

  public default void setAlgaeStatus(boolean status) {}

  public default void stopHere() {}

  public default void stopAll() {}

  public default void zero() {}

  public default void setBrake(boolean lock) {}

  public default void lockRoller() {}
}
