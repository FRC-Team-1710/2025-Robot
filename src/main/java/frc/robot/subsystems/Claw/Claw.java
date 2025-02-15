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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
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

  // Current arm distance mode
  private ClawPosition currentMode = ClawPosition.IDLE;

  private boolean hasAlgae = false;

  // Alerts for motor connection status
  private final Alert wrisAlert = new Alert("Wrist motor isn't connected", AlertType.kError);
  private final Alert clawAlert = new Alert("Claw motor isn't connected", AlertType.kError);

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Claw(ClawIO io) {
    this.io = io;
    this.inputs = new ClawIOInputsAutoLogged();
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);

    // Update motor connection status alerts
    wrisAlert.set(!inputs.wristConnected);
    clawAlert.set(!inputs.clawConnected);

    hasAlgae = inputs.hasAlgae;
  }

  /**
   * Runs the arm in closed-loop distance mode to the specified angle.
   *
   * @param distance The target angle distance
   */
  private void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public void runClaw(double power) {
    io.runClaw(power);
  }

  public void wristManual(double power) {
    io.wristManual(power);
  }

  public void stopHere() {
    io.stopHere();
  }

  /** Stops all motors. */
  private void stop() {
    io.stopAll();
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  /**
   * Returns the current distance of the arm.
   *
   * @return The current angular distance
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.angle;
  }

  /** Enumeration of available arm distances with their corresponding target angles. */
  private enum ClawPosition {
    STOP(Degrees.of(0)), // Stop the wrist
    IDLE(Degrees.of(0), Degrees.of(2.5)), // Wrist tucked in
    REEF(Degrees.of(90), Degrees.of(2.5)), // Position for grabing on reef
    NET(Degrees.of(45), Degrees.of(2.5)); // Position for scoring in net

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ClawPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ClawPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current arm distance mode.
   *
   * @return The current ClawPosition
   */
  public ClawPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new arm distance and schedules the corresponding command.
   *
   * @param mode The desired ClawPosition
   */
  private void setClawPosition(ClawPosition mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current distance
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ClawPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Elevator"),
              ClawPosition.IDLE,
              createPositionCommand(ClawPosition.IDLE),
              ClawPosition.REEF,
              createPositionCommand(ClawPosition.REEF),
              ClawPosition.NET,
              createPositionCommand(ClawPosition.NET)),
          this::getMode);

  /**
   * Creates a command for a specific arm distance that moves the arm and checks the target
   * distance.
   *
   * @param position The arm distance to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ClawPosition position) {
    return Commands.runOnce(() -> setAngle(position.targetAngle))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the arm is at its target distance.
   *
   * @return true if at target distance, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ClawPosition.STOP) return true;
    return getPosition().isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  private Angle targetAngle() {
    return currentMode.targetAngle;
  }

  /**
   * Creates a command to set the arm to a specific distance.
   *
   * @param angle The desired arm distance
   * @return Command to set the distance
   */
  private Command setPositionCommand(ClawPosition angle) {
    return Commands.runOnce(() -> setClawPosition(angle))
        .withName("SetElevatorPosition(" + angle.toString() + ")");
  }

  /** Factory methods for common distance commands */

  /**
   * @return Command to move the arm to L1 scoring distance
   */
  public final Command IDLE() {
    return setPositionCommand(ClawPosition.IDLE);
  }

  /**
   * @return Command to move the arm to L2 scoring distance
   */
  public final Command REEF() {
    return setPositionCommand(ClawPosition.REEF);
  }

  /**
   * @return Command to move the arm to L3 distance
   */
  public final Command NET() {
    return setPositionCommand(ClawPosition.NET);
  }

  /**
   * @return Command to stop the arm
   */
  public final Command stopCommand() {
    return setPositionCommand(ClawPosition.STOP);
  }
}
