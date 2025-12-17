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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.superstructure.claw.Claw.ClawStates;

@Logged
public interface ClawIO {
  @Logged
  public static class ClawIOInputs {
  @Logged(name = "Manual", importance = Importance.INFO)
    public double wristManual = 0.0;
    @Logged(name = "IntakePercent", importance = Importance.INFO)
    public double intakePercent = 0.0;

    @Logged(name = "Locked", importance = Importance.CRITICAL)
    public boolean locked = false;

    @Logged(name = "Setpoint", importance = Importance.CRITICAL)
    public Angle setpoint = Degrees.of(0);

    @Logged(name = "ClawConnected", importance = Importance.CRITICAL)
    public boolean clawConnected = false;
    @Logged(name = "WristConnected", importance = Importance.CRITICAL)
    public boolean wristConnected = false;

    @NotLogged
    public boolean hasZeroed = false;

    @Logged(name = "HasAlgae", importance = Importance.CRITICAL)
    public boolean hasAlgae = false;
    @Logged(name = "RollerLocked", importance = Importance.CRITICAL)
    public boolean rollerLocked = false;

    @Logged(name = "KillSwitch", importance = Importance.CRITICAL)
    public boolean killSwitch = false;

    @Logged(name = "WristMotorAngle", importance = Importance.INFO)
    public Angle wristMotorAngle = Degrees.of(0);
    @Logged(name = "Angle", importance = Importance.CRITICAL)
    public Angle angle = Degrees.of(0);

    @Logged(name = "WristVelocity", importance = Importance.INFO)
    public AngularVelocity wristVelocity = DegreesPerSecond.of(0);
    @Logged(name = "IntakeVelocity", importance = Importance.INFO)
    public AngularVelocity intakeVelocity = DegreesPerSecond.of(0);

    @Logged(name = "WristAppliedVoltage", importance = Importance.INFO)
    public Voltage wristAppliedVoltage = Volts.of(0.0);
    @Logged(name = "WristStatorCurrent", importance = Importance.INFO)
    public Current wristStatorCurrent = Amps.of(0);
    @Logged(name = "WristSupplyCurrent", importance = Importance.INFO)
    public Current wristSupplyCurrent = Amps.of(0);
    @Logged(name = "RollerStatorCurrent", importance = Importance.INFO)
    public Current rollerStatorCurrent = Amps.of(0);
    @Logged(name = "RollerSupplyCurrent", importance = Importance.INFO)
    public Current rollerSupplyCurrent = Amps.of(0);
    @Logged(name = "RollerAppliedVoltage", importance = Importance.INFO)
    public Voltage rollerAppliedVoltage = Volts.of(0.0);

    @Logged(name = "RollerPosition", importance = Importance.CRITICAL)
    public double rollerPosition = 0.0;

    @NotLogged
    public ClawStates state = ClawStates.IDLE;
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

  public default void zeroPIDToAngle() {}

  public default void stopHere() {}

  public default void stopAll() {}

  public default void zero() {}

  public default void setBrake(boolean lock) {}

  public default void lockRoller() {}
}
