// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIOInputsAutoLogged inputs;
  private final ManipulatorIO io;

  private ManipulatorStates currentState = ManipulatorStates.OFF;

  /** Creates a new Claw. */
  public Manipulator(ManipulatorIO io) {
    this.io = io;
    this.inputs = new ManipulatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    switch (currentState) {
      case OFF:
        io.setVoltage(0);
        break;
      case INTAKE:
        if (inputs.beam1Broken && inputs.beam2Broken) {
          io.setVoltage(ManipulatorConstants.insideSpeed * 12);
        } else if (!inputs.beam1Broken && inputs.beam2Broken) {
          io.setVoltage(0);
        } else if (!inputs.beam1Broken && !inputs.beam2Broken) {
          io.setVoltage(ManipulatorConstants.intakeSpeed * 12);
        } else if (inputs.beam1Broken && !inputs.beam2Broken) {
          io.setVoltage(ManipulatorConstants.insideSpeed * 12);
        }
        break;
      case BUMP:
        io.setVoltage(-0.075 * 12);
        break;
      case OUTTAKE:
        io.setVoltage(ManipulatorConstants.outtakeSpeed * 12);
        break;
      default:
        break;
    }
  }

  public enum ManipulatorStates {
    OFF(),
    INTAKE(),
    BUMP(),
    OUTTAKE()
  }

  public void setState(ManipulatorStates state) {
    this.currentState = state;
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return inputs.beam2Broken && !inputs.beam1Broken;
  }

  @AutoLogOutput
  public boolean detectsCoral() {
    return inputs.beam2Broken || inputs.beam1Broken;
  }
}
