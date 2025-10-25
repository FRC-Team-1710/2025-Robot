// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIOInputsAutoLogged inputs;
  private final ManipulatorIO io;

  private ManipulatorStates currentState = ManipulatorStates.OFF;
  private CurrentCoralState currentCoralState = CurrentCoralState.NONE;

  private final BooleanSupplier ejectBoolean;

  /** Creates a new Claw. */
  public Manipulator(ManipulatorIO io, BooleanSupplier bumpBoolean) {
    this.io = io;
    this.inputs = new ManipulatorIOInputsAutoLogged();
    this.ejectBoolean = bumpBoolean;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

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

  @AutoLogOutput
  public boolean hasCoral() {
    return Constants.currentMode == Mode.SIM
        ? (currentCoralState == CurrentCoralState.SECURED)
        : (inputs.beam2Broken && !inputs.beam1Broken);
  }

  @AutoLogOutput
  public boolean almostHasCoral() {
    return Constants.currentMode == Mode.SIM
        ? (currentCoralState == CurrentCoralState.SECURED)
        : (inputs.beam2Broken && inputs.beam1Broken);
  }

  @AutoLogOutput
  public boolean detectsCoral() {
    return Constants.currentMode == Mode.SIM
        ? (currentCoralState == CurrentCoralState.DETECTS
            || currentCoralState == CurrentCoralState.SECURED)
        : (inputs.beam2Broken || inputs.beam1Broken);
  }

  public void advanceGamePiece() {
    currentCoralState =
        switch (currentCoralState) {
          case NONE -> CurrentCoralState.DETECTS;
          case DETECTS -> CurrentCoralState.SECURED;
          case SECURED -> CurrentCoralState.NONE;
        };
  }
}
