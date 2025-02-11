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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls a dual-motor arm mechanism for game piece manipulation. It
 * supports multiple distances for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Claw extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs;

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Claw(ClawIO io) {
    this.io = io;
    this.inputs = new ClawIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);

    io.updatePID(inputs);
  }

  /**
   * Runs the claw in closed-loop position mode to the specified angle.
   *
   * @param distance The target angle (degrees)
   */
  public void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  /**
   * Current position of claw
   *
   * @return Current angle in degrees
   */
  public Angle getAngle() {
    return inputs.angle;
  }

  /**
   * Set the claw angular speed manualy. WARNING: WILL OVERRIDE THE SET ANGLE!!!
   *
   * @param double Power from 1 to -1
   */
  public void setManual(double power) {
    io.setManual(power);
  }

  /**
   * Set the claw intake speed manualy.
   *
   * @param double Power from 1 to -1
   */
  public void runPercent(double power) {
    io.runPercent(power);
  }
}
