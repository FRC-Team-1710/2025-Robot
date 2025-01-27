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

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    public double manualSpin = 0.0;

    public Distance setpoint = Meters.of(0);

    public static Distance SIMsetpoint = Meters.of(0);

    public Angle leaderPosition = Rotations.of(0);
    public Distance distance = Meters.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);

    public Voltage leaderAppliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);
    public Voltage followerAppliedVoltage = Volts.of(0.0);
    public AngularVelocity followerVelocity = RotationsPerSecond.of(0);
    public Angle followerPosition = Rotations.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setDistance(Distance distance) {}

  /** Set power the elevator from 1 to -1 */
  public default void setManual(double power) {}

  /** Resets the state of the PID controller. Used to go from manual power setting to setpoints */
  public default void resetPID() {}

  /**
   * Updates pid loop in ElevatorIOCTRE
   *
   * @param inputs
   */
  public default void updatePID(ElevatorIOInputsAutoLogged inputs) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void resetEncoder() {}
}
