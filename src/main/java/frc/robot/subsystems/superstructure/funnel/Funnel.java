// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and closed-loop control
 * options.
 */
public class Funnel extends SubsystemBase {
  // Hardware interface and inputs
  private final FunnelIO io;
  private final ArmIOInputsAutoLogged inputs;

  // Current arm position mode
  private ArmMode currentMode = ArmMode.INTAKE;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Arm funnelLeader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Arm funnelFollower motor isn't connected", AlertType.kError);
  private final Alert angleMotorAlert =
      new Alert("Arm funnelAngleMotor motor isn't connected", AlertType.kError);

  /**
   * Creates a new Arm subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Funnel(FunnelIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    angleMotorAlert.set(!inputs.angleMotorConnected);
  }

  /**
   * Runs the arm in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  private void setPosition(Angle position) {
    io.setPosition(position);
  }

  public void setRollerPower(double power) {
    io.setRoller(power);
  }

  /**
   * Returns the current position of the arm.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return Degrees.of(inputs.funnelAngle);
  }

  /** Enumeration of available arm positions with their corresponding target angles. */
  private enum ArmMode {
    CLIMB(Degrees.of(105), Degrees.of(2.5)), // Arm fully raised
    INTAKE(Degrees.of(0), Degrees.of(2.5)), // Arm tucked in
    L1(Degrees.of(85), Degrees.of(2.5)); // Position for scoring in L1

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ArmMode(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }
  }

  /**
   * Gets the current arm position mode.
   *
   * @return The current ArmMode
   */
  public ArmMode getMode() {
    return currentMode;
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  /**
   * Sets a new arm mode and schedules the corresponding command.
   *
   * @param mode The desired ArmMode
   */
  private void setArmMode(ArmMode mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ArmMode.CLIMB,
              createPositionCommand(ArmMode.CLIMB),
              ArmMode.INTAKE,
              createPositionCommand(ArmMode.INTAKE),
              ArmMode.L1,
              createPositionCommand(ArmMode.L1)),
          this::getMode);

  /**
   * Creates a command for a specific arm position that moves the arm and checks the target
   * position.
   *
   * @param mode The arm position to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ArmMode mode) {
    return Commands.runOnce(() -> setPosition(mode.targetAngle))
        .withName("Move to " + mode.toString());
  }

  /**
   * Checks if the arm is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
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

  public void setRoller(double percent) {
    io.setRoller(percent);
  }

  public void zero() {
    io.zero();
  }

  /**
   * Creates a command to set the arm to a specific mode.
   *
   * @param mode The desired arm mode
   * @return Command to set the mode
   */
  private Command setPositionCommand(ArmMode mode) {
    return Commands.runOnce(() -> setArmMode(mode)).withName("SetArmMode(" + mode.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to move the arm to L1 scoring position
   */
  public final Command CLIMB() {
    return setPositionCommand(ArmMode.CLIMB);
  }

  /**
   * @return Command to move the arm to L1 scoring position
   */
  public final Command L1() {
    return setPositionCommand(ArmMode.L1);
  }

  /**
   * @return Command to intake the arm
   */
  public final Command intake() {
    return setPositionCommand(ArmMode.INTAKE);
  }
}
