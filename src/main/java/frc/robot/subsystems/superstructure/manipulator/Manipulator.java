// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorIO.ManipulatorIOInputs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

@Logged
public class Manipulator {
  @Logged(name = "Inputs", importance = Importance.CRITICAL)
  private final ManipulatorIOInputs inputs;
  @Logged(name = "IO", importance = Importance.CRITICAL)
  private final ManipulatorIO io;

  @Logged(name = "CurrentState", importance = Importance.CRITICAL)
  private ManipulatorStates currentState = ManipulatorStates.OFF;
  // Only log if debug (if sim)
  @Logged(name = "CurrentCoralState", importance = Importance.DEBUG)
  private CurrentCoralState currentCoralState = CurrentCoralState.NONE;

  @Logged(name = "EjectBoolean", importance = Importance.CRITICAL)
  private final BooleanSupplier ejectBoolean;

  /** Creates a new Claw. */
  public Manipulator(ManipulatorIO io, BooleanSupplier bumpBoolean) {
    this.io = io;
    this.inputs = new ManipulatorIOInputs();
    this.ejectBoolean = bumpBoolean;
  }

  public void periodic() {
    io.updateInputs(inputs);

    if (ejectBoolean.getAsBoolean()) {
      io.setVoltage(ManipulatorConstants.outtakeSpeed * 12);
    } else {
      switch (currentState) {
        case OFF:
          if (hasCoral() || !detectsCoral()) {
            io.setVoltage(0);
          } else if (detectsCoral()) {
            io.setVoltage(ManipulatorConstants.insideSpeed * 12);
          }
          break;
        case INTAKE:
          if (hasCoral()) {
            io.setVoltage(0);
          } else if (!detectsCoral()) {
            io.setVoltage(ManipulatorConstants.intakeSpeed * 12);
          } else if (detectsCoral()) {
            io.setVoltage(ManipulatorConstants.insideSpeed * 12);
          }
          break;
        case OUTTAKE:
          io.setVoltage(ManipulatorConstants.outtakeSpeed * 12);
          break;
        default:
          break;
      }
    }
  }

  public enum ManipulatorStates {
    OFF(),
    INTAKE(),
    OUTTAKE()
  }

  public enum CurrentCoralState {
    NONE(),
    DETECTS(),
    SECURED()
  }

  public void setState(ManipulatorStates state) {
    this.currentState = state;
  }

  @Logged(name = "HasCoral", importance = Importance.INFO)
  public boolean hasCoral() {
    return Constants.currentMode == Mode.SIM
        ? (currentCoralState == CurrentCoralState.SECURED)
        : (inputs.beam2Broken && !inputs.beam1Broken);
  }

  @Logged(name = "AlmostHasCoral", importance = Importance.INFO)
  public boolean almostHasCoral() {
    return Constants.currentMode == Mode.SIM
        ? (currentCoralState == CurrentCoralState.SECURED)
        : (inputs.beam2Broken && inputs.beam1Broken);
  }

  @Logged(name = "DetectsCoral", importance = Importance.INFO)
  public boolean detectsCoral() {
    return Constants.currentMode == Mode.SIM
        ? (currentCoralState == CurrentCoralState.DETECTS
            || currentCoralState == CurrentCoralState.SECURED)
        : (inputs.beam2Broken || inputs.beam1Broken);
  }

  @Logged(name = "CanElevatorMove", importance = Importance.CRITICAL)
  public boolean canElevatorMove() {
    // return Constants.currentMode == Mode.SIM
    //     ? (currentCoralState == CurrentCoralState.DETECTS
    //         && currentCoralState != CurrentCoralState.SECURED)
    // : (!inputs.beam1Broken);
    return Constants.currentMode == Mode.SIM ? hasCoral() : !inputs.beam1Broken;
  }

  public void advanceGamePiece() {
    if (Constants.currentMode == Mode.SIM) {
      currentCoralState =
          switch (currentCoralState) {
            case NONE -> CurrentCoralState.DETECTS;
            case DETECTS -> CurrentCoralState.SECURED;
            case SECURED -> CurrentCoralState.NONE;
          };
    }
  }
}
