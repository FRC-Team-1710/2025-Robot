// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface FunnelIO {
  @Logged
  public static class FunnelIOInputs {
    @Logged(name = "LeaderConnected", importance = Importance.CRITICAL)
    public boolean leaderConnected = false;

    @Logged(name = "FollowerConnected", importance = Importance.CRITICAL)
    public boolean followerConnected = false;

    @Logged(name = "AngleConnected", importance = Importance.CRITICAL)
    public boolean angleMotorConnected = false;

    @Logged(name = "HasCoral", importance = Importance.INFO)
    public boolean hasCoral = false;

    @Logged(name = "LeaderPosition", importance = Importance.INFO)
    public Angle leaderPosition = Rotations.of(0);

    @Logged(name = "LeaderRotorPosition", importance = Importance.INFO)
    public Angle leaderRotorPosition = Rotations.of(0);

    @Logged(name = "AngleMotorPosition", importance = Importance.INFO)
    public Angle angleMotorPosition = Rotations.of(0);

    @Logged(name = "LeaderVelocity", importance = Importance.INFO)
    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);

    @Logged(name = "FollowerVelocity", importance = Importance.INFO)
    public AngularVelocity followerVelocity = RotationsPerSecond.of(0);

    @Logged(name = "AngleMotorVelocity", importance = Importance.INFO)
    public AngularVelocity angleMotorVelocity = RotationsPerSecond.of(0);

    @Logged(name = "AppliedVoltage", importance = Importance.INFO)
    public Voltage appliedVoltage = Volts.of(0.0);

    @Logged(name = "LeaderStatorCurrent", importance = Importance.INFO)
    public Current leaderStatorCurrent = Amps.of(0);

    @Logged(name = "FollowerStatorCurrent", importance = Importance.INFO)
    public Current followerStatorCurrent = Amps.of(0);

    @Logged(name = "AngleMotorStatorCurrent", importance = Importance.INFO)
    public Current angleMotorStatorCurrent = Amps.of(0);

    @Logged(name = "LeaderSupplyCurrent", importance = Importance.INFO)
    public Current leaderSupplyCurrent = Amps.of(0);

    @Logged(name = "FollowerSupplyCurrent", importance = Importance.INFO)
    public Current followerSupplyCurrent = Amps.of(0);

    @Logged(name = "AngleMotorSupplyCurrent", importance = Importance.INFO)
    public Current angleMotorSupplyCurrent = Amps.of(0);

    @Logged(name = "FunnelAngle", importance = Importance.CRITICAL)
    public Angle funnelAngle = Degrees.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FunnelIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(Angle angle) {}

  /** setRoller */
  public default void setRoller(double percent) {}

  public default void zero() {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void setAileron(double setpoint) {}
}
