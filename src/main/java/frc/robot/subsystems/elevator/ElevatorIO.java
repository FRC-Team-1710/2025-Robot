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
import static edu.wpi.first.units.Units.Inches;
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
  public class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    public Angle leaderPosition = Rotations.of(0);
    public Angle leaderRotorPosition = Rotations.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity leaderRotorVelocity = RotationsPerSecond.of(0);

    public Angle followerPosition = Rotations.of(0);
    public Angle followerRotorPosition = Rotations.of(0);

    public AngularVelocity followerVelocity = RotationsPerSecond.of(0);
    public AngularVelocity followerRotorVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);

    public Distance elevatorDistance = Inches.of(0);

    public Distance setPoint = Inches.of(0);
    public double manual = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setDistance(Distance distance) {}

  public default void setManual(double power) {}

  public default void stopHere() {}

  /** Stop in open loop. */
  public default void stop() {}
}
