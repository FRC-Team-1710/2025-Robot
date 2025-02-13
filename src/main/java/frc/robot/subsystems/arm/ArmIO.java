// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public boolean encoderConnected = false;

    public Angle leaderPosition = Rotations.of(0);
    public Angle leaderRotorPosition = Rotations.of(0);
    public Angle encoderPosition = Rotations.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity leaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity encoderVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);

    public Angle armAngle = Rotations.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(Angle angle) {}

  /** Stop in open loop. */
  public default void stop() {}
}
