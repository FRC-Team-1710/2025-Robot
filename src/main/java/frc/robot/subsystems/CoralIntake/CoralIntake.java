// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  private final CoralIntakeIO io;
  private final CoralIntakeIOInputsAutoLogged inputs;
  /** Creates a new Claw. */
  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
    this.inputs = new CoralIntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claws", inputs);
  }

  // Sim
  public void runPercent(double percent) {
    SmartDashboard.putNumber("Claw/ClawPercent", percent);
    io.setVoltage(percent * 12);
  }

  /*Boolean */
  public boolean beam1Broken() {
    SmartDashboard.putBoolean("CoralIntake/Beam1 Broken?", inputs.beam1Broken);
    return inputs.beam1Broken;
  }

  /*Boolean */
  public boolean beam2Broken() {
    SmartDashboard.putBoolean("Coral/Beam2 Broken?", inputs.beam2Broken);
    return inputs.beam2Broken;
  }
}
