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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Claw subsystem controls a single-motor claw mechanism for game piece
 * manipulation. It
 * supports multiple angles for different game actions and provides both
 * open-loop and closed-loop
 * control options.
 */
public class Claw extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs;

  // Current claw angle mode
  private ClawStates currentState = ClawStates.IDLE;

  private boolean doneZeroing = false;

  private Timer timer = new Timer();

  private static boolean hasAlgae = false;
  public boolean rollerLocked;
  private double rollerPositionWhenAlgaeGrabbed = 0;

  // Alerts for motor connection status
  private final Alert wristAlert = new Alert("Wrist motor isn't connected", AlertType.kError);
  private final Alert clawAlert = new Alert("Claw motor isn't connected", AlertType.kError);

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io
   *          The hardware interface implementation for the claw
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

    if (inputs.hasAlgae && Math.abs(rollerPositionWhenAlgaeGrabbed - inputs.rollerPosition) > 1.6) {
      inputs.hasAlgae = false;
    }

    Logger.recordOutput("Claw/State", getState());

    rollerLocked = inputs.rollerLocked;

    switch (currentState) {
      case STOP:
        io.stopAll();
        break;
      case ZERO:
        if (doneZeroing) {
          io.setAngle(ClawStates.IDLE.targetAngle);
        } else {
          io.wristManual(-0.5);
          doneZeroing = (timer.hasElapsed(0.25) && inputs.wristStatorCurrent.in(Amps) > 18);
        }
        break;
      case IDLE:
        io.setAngle(currentState.targetAngle);
        break;
      case HOLD:
        io.setAngle(currentState.targetAngle);
        break;
      case GRAB:
        if (inputs.hasAlgae) {
          io.setAngle(ClawStates.HOLD.targetAngle);
        } else {
          io.setAngle(currentState.targetAngle);
          io.setRollers(0.5);
          inputs.hasAlgae = (timer.hasElapsed(0.25) && inputs.rollerStatorCurrent.in(Amps) > 120);
        }
        break;
      case NET:
        io.setAngle(currentState.targetAngle);
        break;
      case FLOOR:
        io.setAngle(currentState.targetAngle);
        break;
      case PROCESSOR:
        io.setAngle(currentState.targetAngle);
        break;
      default:
        break;
    }
  }

  /**
   * Enumeration of available claw angles with their corresponding target angles.
   */
  public enum ClawStates {
    STOP(Degrees.of(0)), // Stop the wrist
    ZERO(Degrees.of(0)),
    IDLE(Degrees.of(2), Degrees.of(2.5)), // Wrist tucked in
    GRAB(Degrees.of(90), Degrees.of(2.5)), // Position for grabing algae
    HOLD(Degrees.of(20), Degrees.of(2.5)), // Position for holding algae
    NET(Degrees.of(20), Degrees.of(2.5)), // Position for scoring in net
    FLOOR(Degrees.of(110), Degrees.of(2.5)),
    PROCESSOR(Degrees.of(90));

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ClawStates(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ClawStates(Angle targetAngle) {
      this(targetAngle, Degrees.of(2.5));
    }
  }

  /**
   * Gets the current claw angle mode.
   *
   * @return The current ClawPosition
   */
  public ClawStates getState() {
    return currentState;
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentState == ClawStates.STOP)
      return true;
    return inputs.angle.isNear(currentState.targetAngle, currentState.angleTolerance);
  }

  public void setState(ClawStates state) {
    if(state != ClawStates.ZERO) doneZeroing = false;
    if (state != ClawStates.ZERO && state != ClawStates.GRAB) {
      timer.reset();
    }
    if (!timer.isRunning()) timer.start();
    this.currentState = state;
  }
}
