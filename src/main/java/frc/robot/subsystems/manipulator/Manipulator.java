// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIOInputsAutoLogged inputs;
  private final ManipulatorIO io;

  private States state = States.Off;

  /** Creates a new Claw. */
  public Manipulator(ManipulatorIO io) {
    this.io = io;
    this.inputs = new ManipulatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    if (state == States.Intake) {
      intake();
    } else if (isCoralIn() && !isCoralSecure() && state != States.Place) {
      intake();
    } else {
      runPercent(state.placeSpeed);
    }
  }

  public void Off() {
    this.state = States.Off;
  }

  public void Intake() {
    this.state = States.Intake;
  }

  public void Place() {
    this.state = States.Place;
  }

  public enum States {
    Intake(0.5, 0.25),
    Place(0.25),
    Off;

    private final double OUTSIDEpercent;
    private final double INSIDEpercent;
    private final double placeSpeed;

    States() {
      this.OUTSIDEpercent = 0;
      this.INSIDEpercent = 0;
      this.placeSpeed = 0;
    }

    States(double placeSpeed) {
      this.OUTSIDEpercent = 0;
      this.INSIDEpercent = 0;
      this.placeSpeed = placeSpeed;
    }

    States(double OUTSIDEpercent, double INSIDEpercent) {
      this.OUTSIDEpercent = OUTSIDEpercent;
      this.INSIDEpercent = INSIDEpercent;
      this.placeSpeed = 0.0;
    }
  }

  private void intake() {
    if (isCoralIn()) {
      runPercent(States.Intake.INSIDEpercent);
    } else if (isCoralSecure()) {
      runPercent(0);
    } else {
      runPercent(States.Intake.OUTSIDEpercent);
    }
  }

  private void runPercent(double percent) {
    io.setVoltage(percent * 12);
  }

  public boolean isCoralIn() {
    if (!beam1Broken() && !beam2Broken()) {
      return false;
    } else {
      return true;
    }
  }

  public boolean isCoralSecure() {
    if (!beam1Broken() && beam2Broken()) {
      return true;
    } else {
      return false;
    }
  }

  /*Boolean */
  public boolean beam1Broken() {
    return inputs.beam1Broken;
  }

  /*Boolean */
  public boolean beam2Broken() {
    return inputs.beam2Broken;
  }
}
