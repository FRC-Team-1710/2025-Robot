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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Claw subsystem controls a single-motor claw mechanism for game piece manipulation. It
 * supports multiple angles for different game actions and provides both open-loop and closed-loop
 * control options.
 */
public class Claw extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs;

  private ClawStates currentState = ClawStates.IDLE;
  private CurrentAlgaeState currentAlgaeState = CurrentAlgaeState.NONE;

  private boolean doneZeroing = false;
  private double rollerPositionWhenAlgaeGrabbed = 0;

  private Timer timer = new Timer();

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
    // SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
    Logger.recordOutput("Claw/State", getState());

    wristAlert.set(!inputs.wristConnected);
    clawAlert.set(!inputs.clawConnected);

    if (inputs.hasAlgae
        && Math.abs(rollerPositionWhenAlgaeGrabbed - inputs.rollerPosition) > 1.6
        && Constants.currentMode != Mode.SIM) {
      inputs.hasAlgae = false;
    } else if (Constants.currentMode == Mode.SIM) {
      inputs.hasAlgae = currentAlgaeState == CurrentAlgaeState.HAS_ALGAE;
    }

    if (!inputs.hasAlgae
        && currentState != ClawStates.GRAB
        && currentState != ClawStates.FLOOR
        && currentState != ClawStates.SCORE_NET
        && currentState != ClawStates.SCORE_PROCESSOR) {
      io.setRollers(0);
    }

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
        if (!inputs.hasAlgae) {
          io.setAngle(currentState.targetAngle);
          io.setRollers(0.5);
          if (Constants.currentMode != Mode.SIM) {
            inputs.hasAlgae = (timer.hasElapsed(0.25) && inputs.rollerStatorCurrent.in(Amps) > 120);
          }
          if (inputs.hasAlgae) {
            rollerPositionWhenAlgaeGrabbed = inputs.rollerPosition;
            io.lockRoller();
          }
        }
        break;
      case NET:
        io.setAngle(currentState.targetAngle);
        break;
      case SCORE_NET:
        io.setAngle(currentState.targetAngle);
        io.setRollers(-0.2);
        break;
      case FLOOR:
        if (inputs.hasAlgae) {
          io.setAngle(ClawStates.HOLD.targetAngle);
        } else {
          io.setAngle(currentState.targetAngle);
          io.setRollers(0.5);
          if (Constants.currentMode != Mode.SIM) {
            inputs.hasAlgae = (timer.hasElapsed(0.25) && inputs.rollerStatorCurrent.in(Amps) > 120);
          }
          if (hasAlgae()) {
            rollerPositionWhenAlgaeGrabbed = inputs.rollerPosition;
            io.lockRoller();
          }
        }
        break;
      case PROCESSOR:
        io.setAngle(currentState.targetAngle);
        break;
      case SCORE_PROCESSOR:
        io.setAngle(currentState.targetAngle);
        io.setRollers(-0.2);
        break;
      default:
        break;
    }
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }

  public boolean isDoneZeroing() {
    return doneZeroing;
  }

  /** Enumeration of available claw angles with their corresponding target angles. */
  public enum ClawStates {
    STOP(Degrees.of(0)), // Stop the wrist
    ZERO(Degrees.of(0)),
    IDLE(Degrees.of(2), Degrees.of(2.5)), // Wrist tucked in
    GRAB(Degrees.of(90), Degrees.of(2.5)), // Position for grabing algae
    HOLD(Degrees.of(20), Degrees.of(2.5)), // Position for holding algae
    NET(Degrees.of(20), Degrees.of(2.5)), // Position for scoring in net
    SCORE_NET(Degrees.of(20), Degrees.of(2.5)), // Position for scoring in net
    FLOOR(Degrees.of(110), Degrees.of(2.5)),
    PROCESSOR(Degrees.of(90)),
    SCORE_PROCESSOR(Degrees.of(90));

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

  public ClawStates getState() {
    return currentState;
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentState == ClawStates.STOP) return true;
    return inputs.angle.isNear(currentState.targetAngle, currentState.angleTolerance);
  }

  public void setState(ClawStates state) {
    if (state != ClawStates.ZERO) doneZeroing = false;
    if (state != ClawStates.ZERO && state != ClawStates.GRAB) {
      timer.reset();
    }
    if (!timer.isRunning()) timer.start();
    this.currentState = state;
  }

  public void advanceGamePiece() {
    currentAlgaeState =
        switch (currentAlgaeState) {
          case NONE -> CurrentAlgaeState.HAS_ALGAE;
          case HAS_ALGAE -> CurrentAlgaeState.NONE;
        };
  }

  public enum CurrentAlgaeState {
    NONE(),
    HAS_ALGAE(),
  }
}
