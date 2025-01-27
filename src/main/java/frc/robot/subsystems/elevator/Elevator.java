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

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Elevator subsystem controls a dual-motor arm mechanism for game piece manipulation. It
 * supports multiple distances for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Elevator extends SubsystemBase {
  // Hardware interface and inputs
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private Distance selectedLevel = Meters.of(0);

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Elevator leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Elevator follower motor isn't connected", AlertType.kError);

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    io.updatePID(inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
  }

  /**
   * Runs the elevator in closed-loop distance mode to the specified height.
   *
   * @param distance The target distance (meters)
   */
  public void setDistance(Distance distance) {
    io.setDistance(distance);
  }

  /**
   * Runs the elevator in closed-loop distance mode to the specified height.
   *
   * @return Current distance in meters
   */
  public Distance getDistance() {
    return inputs.distance;
  }

  /** Stops the elevator */
  public void stop() {
    io.stop();
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  /**
   * Preselects target distance for targeting different branch heights. It will not automatically go
   * there! (Meters)
   */
  public void setTargetDistance(Distance target) {
    selectedLevel = target;
  }

  /** Returns set distance for current selected level */
  public Distance getTargetDistance() {
    return selectedLevel;
  }

  /**
   * Set the elevator speed manualy. WARNING: WILL OVERRIDE THE SET DISTANCE!!!
   *
   * @param double Power from 1 to -1
   */
  public void setManual(double power) {
    io.setManual(power);
  }

  /** Resets the state of the PID controller. Used to go from manual power setting to setpoints */
  public void resetPID() {
    io.resetPID();
  }
}
