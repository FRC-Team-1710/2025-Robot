// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.climber.Climber;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualClimb extends Command {
  private double power;
  private Climber m_Climber;
  private double upperLimit = 180;
  private double lowerLimit = 0.1;

  /** Creates a new ManualClimb. */
  public ManualClimb(Climber m_Climber, DoubleSupplier power) {
    this.m_Climber = m_Climber;
    this.power = power.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((0 < power && m_Climber.getPosition() < upperLimit)
        || (power < 0 && lowerLimit < m_Climber.getPosition() || power == 0)) {
      m_Climber.SetClimberPower(power);
    } else {
      m_Climber.SetClimberPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climber.SetClimberPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
