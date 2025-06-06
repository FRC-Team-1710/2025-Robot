// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIOInputsAutoLogged inputs;
  private final ManipulatorIO io;

  private static boolean hasCoral;

  /** Creates a new Claw. */
  public Manipulator(ManipulatorIO io) {
    SmartDashboard.putNumber("Coral/x", 0);
    SmartDashboard.putNumber("Coral/y", 0);
    SmartDashboard.putNumber("Coral/z", 0);
    SmartDashboard.putNumber("Coral/roll", 0);
    SmartDashboard.putNumber("Coral/pitch", 0);
    SmartDashboard.putNumber("Coral/yaw", 0);
    SimCoral.start();
    this.io = io;
    this.inputs = new ManipulatorIOInputsAutoLogged();
    hasCoral = beam2Broken() && !beam1Broken();
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

  @AutoLogOutput
  public boolean hasCoral() {
    if (Constants.currentMode == Constants.Mode.SIM) return hasCoral;
    else return beam2Broken() && !beam1Broken();
  }

  public void toggleCoralStatus() {
    hasCoral = !hasCoral;
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
