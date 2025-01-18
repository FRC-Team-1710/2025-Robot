// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utils.TunableController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private CoralIntake m_coralIntake;
  private Elevator m_elevator;
  private TunableController controller;

  /** Creates a new IntakeCoral. */
  public IntakeCoral(CoralIntake intake, Elevator ele, TunableController control) {
    this.m_coralIntake = intake;
    this.m_elevator = ele;
    this.controller = control;
    addRequirements(intake, ele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setDistance(ElevatorConstants.Intake);
    m_coralIntake.runPercent(CoralIntakeConstants.intakeSpeed);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_coralIntake.beam1Broken() && m_coralIntake.beam2Broken()) {
      m_coralIntake.runPercent(CoralIntakeConstants.insideSpeed);
      controller.setRumble(RumbleType.kBothRumble, 0.25);
    } else if (!m_coralIntake.beam1Broken() && m_coralIntake.beam2Broken()) {
      m_coralIntake.runPercent(0);
      controller.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralIntake.runPercent(0);
    controller.setRumble(RumbleType.kBothRumble, 0);
    m_elevator.setDistance(Meters.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
