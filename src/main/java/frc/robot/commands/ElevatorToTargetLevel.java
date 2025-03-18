// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.utils.TargetingComputer;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToTargetLevel extends Command {
  Elevator elevator;

  /** Creates a new ElevatorToTargetLevel. */
  public ElevatorToTargetLevel(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Target Level (Command)", TargetingComputer.getCurrentTargetLevel());

    switch (TargetingComputer.getCurrentTargetLevel()) {
      case L1:
        elevator.L1().schedule();
        break;
      case L2:
        elevator.L2().schedule();
        break;
      case L3:
        elevator.L3().schedule();
        break;
      case L4:
        elevator.L4().schedule();
        break;
      case ALGAE_LOW:
        elevator.AlgaeLow().schedule();
        break;
      case ALGAE_HIGH:
        elevator.AlgaeHigh().schedule();
        break;
      case INTAKE:
        elevator.intake().schedule();
        break;
      default:
        elevator.intake().schedule();
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
