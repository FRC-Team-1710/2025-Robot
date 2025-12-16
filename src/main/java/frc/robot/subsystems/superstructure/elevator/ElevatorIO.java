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

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ElevatorIO {
  @Logged
  public class ElevatorIOInputs {
    @Logged(name = "LeaderConnected", importance = Importance.CRITICAL)
    public boolean leaderConnected = false;
    @Logged(name = "FollowerConnected", importance = Importance.CRITICAL)
    public boolean followerConnected = false;

    @Logged(name = "KillSwitch", importance = Importance.CRITICAL)
    public boolean killSwitch = false;
    @Logged(name = "Locked", importance = Importance.CRITICAL)
    public boolean locked = false;

    @Logged(name = "LeaderPosition", importance = Importance.CRITICAL)
    public Angle leaderPosition = Rotations.of(0);

    @Logged(name = "LeaderVelocity", importance = Importance.CRITICAL)
    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);

    @Logged(name = "FollowerPosition", importance = Importance.CRITICAL)
    public Angle followerPosition = Rotations.of(0);

    @Logged(name = "FollowerVelocity", importance = Importance.CRITICAL)
    public AngularVelocity followerVelocity = RotationsPerSecond.of(0);

    @Logged(name = "AppliedVoltage", importance = Importance.CRITICAL)
    public Voltage appliedVoltage = Volts.of(0.0);
    @Logged(name = "LeaderStatorCurrent", importance = Importance.CRITICAL)
    public Current leaderStatorCurrent = Amps.of(0);
    @Logged(name = "FollowerStatorCurrent", importance = Importance.CRITICAL)
    public Current followerStatorCurrent = Amps.of(0);
    @Logged(name = "LeaderSupplyCurrent", importance = Importance.INFO)
    public Current leaderSupplyCurrent = Amps.of(0);
    @Logged(name = "FollowerSupplyCurrent", importance = Importance.INFO)
    public Current followerSupplyCurrent = Amps.of(0);

    @Logged(name = "Distance", importance = Importance.CRITICAL)
    public Distance distance = Inches.of(0);
    @Logged(name = "Goal", importance = Importance.CRITICAL)
    public Distance goal = Inches.of(0);
    @Logged(name = "Setpoint", importance = Importance.CRITICAL)
    public Distance setpoint = Inches.of(0);

    @Logged(name = "Manual", importance = Importance.INFO)
    public double manual = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setDistance(Distance distance) {}

  public default void setManual(double power) {}

  public default void stopHere() {}

  public default void zero() {}

  /** Stop in open loop. */
  public default void stop() {}
}
