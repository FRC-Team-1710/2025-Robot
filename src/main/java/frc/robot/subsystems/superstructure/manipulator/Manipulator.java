// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIOInputsAutoLogged inputs;
  private final ManipulatorIO io;

  public static boolean hasCoral;

  /** Creates a new Claw. */
  public Manipulator(ManipulatorIO io) {
    this.io = io;
    this.inputs = new ManipulatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  // Sim
  public void runPercent(double percent) {
    SmartDashboard.putNumber("Manipulator/ClawPercent", percent);
    io.setVoltage(percent * 12);
  }

  /*Boolean */
  public boolean beam1Broken() {
    SmartDashboard.putBoolean("Manipulator/Beam1 Broken?", inputs.beam1Broken);
    return inputs.beam1Broken;
  }

  /*Boolean */
  public boolean beam2Broken() {
    SmartDashboard.putBoolean("Manipulator/Beam2 Broken?", inputs.beam2Broken);
    return inputs.beam2Broken;
  }
}
