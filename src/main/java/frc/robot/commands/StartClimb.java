// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.climber.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StartClimb extends Command {
  Climber climber;
  double targetRotations = 2.3;

  /** Creates a new Climb. */
  public StartClimb(Climber climber) {
    this.climber = new Climber();
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.SetClimberPower(.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.SetClimberPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getPosition() > targetRotations;
  }
}
