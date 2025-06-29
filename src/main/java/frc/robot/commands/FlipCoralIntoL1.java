// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlipCoralIntoL1 extends Command {
  Elevator elevator;
  Manipulator manipulator;

  /** Creates a new FlipCoralIntoL1. */
  public FlipCoralIntoL1(Elevator elevator, Manipulator manipulator) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.L1().schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.isAtTarget()) {
      manipulator.runPercent(ManipulatorConstants.outtakeSpeed);
      elevator.L2().schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.runPercent(0);
    elevator.INTAKE().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getMode() == ElevatorStates.L2 && elevator.isAtTarget();
  }
}
