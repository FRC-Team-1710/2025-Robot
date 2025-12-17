// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.funnel.FunnelIO.FunnelIOInputs;
import java.util.function.BooleanSupplier;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and closed-loop control
 * options.
 */
@Logged
public class Funnel {
  // Hardware interface and inputs
  @Logged(name = "IO", importance = Importance.CRITICAL)
  private final FunnelIO io;

  @Logged(name = "Inputs", importance = Importance.CRITICAL)
  private final FunnelIOInputs inputs;

  @Logged(name = "CurrentState", importance = Importance.CRITICAL)
  private FunnelState currentState = FunnelState.INTAKE;

  @Logged(name = "Bump", importance = Importance.CRITICAL)
  private final BooleanSupplier bumpBoolean;

  public Funnel(FunnelIO io, BooleanSupplier bumpBoolean) {
    this.io = io;
    this.inputs = new FunnelIOInputs();
    this.bumpBoolean = bumpBoolean;
  }

  public void periodic() {
    io.updateInputs(inputs);

    io.setPosition(currentState.targetAngle);

    if (bumpBoolean.getAsBoolean()) {
      io.setRoller(-0.125);
    } else {
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
        case L1:
          io.setRoller(0);
          break;
        case STOP:
          io.stop();
          break;
        default:
          break;
      }
    }
  }

  @NotLogged
  public Angle getPosition() {
    return inputs.funnelAngle;
  }

  /** Enumeration of available arm positions with their corresponding target angles. */
  public enum FunnelState {
    STOP(Degrees.of(0), Degrees.of(2.5)), // Arm fully raised
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

  @NotLogged
  public FunnelState getState() {
    return currentState;
  }

  public void setState(FunnelState state) {
    this.currentState = state;
  }

  @Logged(name = "HasCoral", importance = Importance.INFO)
  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  @Logged(name = "AtTarget", importance = Importance.CRITICAL)
  public boolean isAtTarget() {
    return getPosition().isNear(currentState.targetAngle, currentState.angleTolerance);
  }
}
