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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece
 * manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and
 * closed-loop control
 * options.
 */
public class Funnel extends SubsystemBase {
  // Hardware interface and inputs
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs;

  private FunnelState currentState = FunnelState.INTAKE;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert = new Alert("Arm funnelLeader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert = new Alert("Arm funnelFollower motor isn't connected", AlertType.kError);
  private final Alert angleMotorAlert = new Alert("Arm funnelAngleMotor motor isn't connected", AlertType.kError);

  public Funnel(FunnelIO io) {
    this.io = io;
    this.inputs = new FunnelIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    angleMotorAlert.set(!inputs.angleMotorConnected);

    io.setPosition(currentState.targetAngle);

    switch (currentState) {
      case CLIMB:
        io.setRoller(0);
        break;
      case OFF:
        io.setRoller(0);
        break;
      case INTAKE:
        io.setRoller(FunnelConstants.intakeSpeed);
        break;
      case INTAKE_SLOW:
        io.setRoller(FunnelConstants.insideSpeed);
        break;
      case BUMP:
        io.setRoller(-0.075);
        break;
      case L1:
        io.setRoller(0);
        break;
      default:
        break;
    }
  }

  @AutoLogOutput
  public Angle getPosition() {
    return Degrees.of(inputs.funnelAngle);
  }

  /**
   * Enumeration of available arm positions with their corresponding target
   * angles.
   */
  public enum FunnelState {
    CLIMB(Degrees.of(100), Degrees.of(2.5)), // Arm fully raised
    OFF(Degrees.of(0), Degrees.of(2.5)), // Arm tucked in
    INTAKE(Degrees.of(0), Degrees.of(2.5)), // Arm tucked in
    INTAKE_SLOW(Degrees.of(0), Degrees.of(2.5)), // Arm tucked in
    BUMP(Degrees.of(0), Degrees.of(2.5)), // Arm tucked in
    L1(Degrees.of(50), Degrees.of(2.5)); // Position for scoring in L1

    private final Angle targetAngle;
    private final Angle angleTolerance;

    FunnelState(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }
  }

  public FunnelState getState() {
    return currentState;
  }

  public void setState(FunnelState state) {
    this.currentState = state;
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    return getPosition().isNear(currentState.targetAngle, currentState.angleTolerance);
  }
}
