// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.utils.TunableController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private Manipulator m_Manipulator;
  private Elevator m_elevator;
  private TunableController controller;

  /** Creates a new IntakeCoral. */
  public IntakeCoral(Manipulator manipulator, Elevator ele, TunableController control) {
    this.m_Manipulator = manipulator;
    this.m_elevator = ele;
    this.controller = control;
    addRequirements(manipulator, ele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_elevator.setDistance(ElevatorConstants.Intake);
    m_Manipulator.runPercent(ManipulatorConstants.intakeSpeed);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()) {
      m_Manipulator.runPercent(ManipulatorConstants.insideSpeed);
      controller.setRumble(RumbleType.kBothRumble, 0.1);
    } else if (!m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()) {
      m_Manipulator.runPercent(0);
      controller.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Manipulator.runPercent(0);
    controller.setRumble(RumbleType.kBothRumble, 0);
    // m_elevator.setDistance(Meters.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
