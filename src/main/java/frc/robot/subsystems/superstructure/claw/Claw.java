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
 * The Claw subsystem controls a single-motor claw mechanism for game piece manipulation. It
 * supports multiple angles for different game actions and provides both open-loop and closed-loop
 * control options.
 */
public class Claw extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs;

  // Current claw angle mode
  private ClawPosition currentMode = ClawPosition.IDLE;

  private static boolean hasAlgae = false;
  public boolean rollerLocked;
  private double rollerPositionWhenAlgaeGrabbed = 0;

  // Alerts for motor connection status
  private final Alert wristAlert = new Alert("Wrist motor isn't connected", AlertType.kError);
  private final Alert clawAlert = new Alert("Claw motor isn't connected", AlertType.kError);

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the claw
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
    wristAlert.set(!inputs.wristConnected);
    clawAlert.set(!inputs.clawConnected);

    // hasAlgae = inputs.hasAlgae;

    if (hasAlgae && Math.abs(rollerPositionWhenAlgaeGrabbed - getRollerPosition()) > 2) {
      hasAlgae = false;
    }

    Logger.recordOutput("Claw/hasAlgae", hasAlgae);
    Logger.recordOutput("mode", Math.abs(rollerPositionWhenAlgaeGrabbed - getRollerPosition()) > 1);

    rollerLocked = inputs.rollerLocked;
  }

  /**
   * Runs the claw in closed-loop angle mode to the specified angle.
   *
   * @param angle The target angle
   */
  private void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public void setRollers(double power) {
    io.setRollers(power);
  }

  public void setAlgaeStatus(boolean status) {
    hasAlgae = status;
  }

  public void toggleAlgaeStatus() {
    hasAlgae = !hasAlgae;
  }

  public double getRollerCurrent() {
    return inputs.rollerStatorCurrent.baseUnitMagnitude();
  }

  public double getWristCurrent() {
    return inputs.wristStatorCurrent.baseUnitMagnitude();
  }

  public double getRollerPosition() {
    return inputs.rollerPosition;
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

  public void setRollerPositionWhenAlgaeGrabbed(double position) {
    rollerPositionWhenAlgaeGrabbed = position;
  }

  /** Zeros wrist */
  public void zero() {
    io.zero();
  }

  public void lockRoller() {
    io.lockRoller();
  }

  /**
   * Returns the current angle of the claw.
   *
   * @return The current angle
   */
  @AutoLogOutput
  public Angle getAngle() {
    return inputs.angle;
  }

  /** Enumeration of available claw angles with their corresponding target angles. */
  private enum ClawPosition {
    STOP(Degrees.of(0)), // Stop the wrist
    IDLE(Degrees.of(0), Degrees.of(2.5)), // Wrist tucked in
    GRAB(Degrees.of(85), Degrees.of(2.5)), // Position for grabing algae
    HOLD(Degrees.of(35), Degrees.of(2.5)), // Position for holding algae
    NET(Degrees.of(35), Degrees.of(2.5)), // Position for scoring in net
    PROCESSOR(Degrees.of(110));

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
   * Gets the current claw angle mode.
   *
   * @return The current ClawPosition
   */
  public ClawPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new claw angle and schedules the corresponding command.
   *
   * @param mode The desired ClawPosition
   */
  private void setClawPosition(ClawPosition mode) {
    if (mode == ClawPosition.IDLE) mode = hasAlgae ? ClawPosition.HOLD : ClawPosition.IDLE;
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current angle
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ClawPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Elevator"),
              ClawPosition.IDLE,
              createPositionCommand(ClawPosition.IDLE),
              ClawPosition.GRAB,
              createPositionCommand(ClawPosition.GRAB),
              ClawPosition.HOLD,
              createPositionCommand(ClawPosition.HOLD),
              ClawPosition.NET,
              createPositionCommand(ClawPosition.NET),
              ClawPosition.PROCESSOR,
              createPositionCommand(ClawPosition.PROCESSOR)),
          this::getMode);

  /**
   * Creates a command for a specific claw angle that moves the claw and checks the target angle.
   *
   * @param position The claw angle to create a command for
   * @return A command that implements the claw movement
   */
  private Command createPositionCommand(ClawPosition position) {
    return Commands.runOnce(() -> setAngle(position.targetAngle))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the claw is at its target angle.
   *
   * @return true if at target angle, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ClawPosition.STOP) return true;
    return getAngle().isNear(currentMode.targetAngle, currentMode.angleTolerance);
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
   * Creates a command to set the claw to a specific angle.
   *
   * @param angle The desired claw angle
   * @return Command to set the angle
   */
  private Command setPositionCommand(ClawPosition angle) {
    return Commands.runOnce(() -> setClawPosition(angle))
        .withName("SetElevatorPosition(" + angle.toString() + ")");
  }

  /** Factory methods for common angle commands */

  /**
   * @return Command to move the claw to idling angle
   */
  public final Command IDLE() {
    return setPositionCommand(ClawPosition.IDLE);
  }

  /**
   * @return Command to move the claw to grabbing angle
   */
  public final Command GRAB() {
    return setPositionCommand(ClawPosition.GRAB);
  }

  /**
   * @return Command to move the claw to algae-holding angle
   */
  public final Command HOLD() {
    return setPositionCommand(ClawPosition.HOLD);
  }

  /**
   * @return Command to move the claw to net angle
   */
  public final Command NET() {
    return setPositionCommand(ClawPosition.NET);
  }

  /**
   * @return Command to move the claw to processor angle
   */
  public final Command PROCESSOR() {
    return setPositionCommand(ClawPosition.PROCESSOR);
  }

  /**
   * @return Command to stop the claw
   */
  public final Command stopCommand() {
    return setPositionCommand(ClawPosition.STOP);
  }
}
