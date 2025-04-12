// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevationManual extends Command {
  private Elevator m_elevatorSubsystem;
  double m_speed;
  DoubleSupplier axis;
  boolean locked = false;

  public ElevationManual(Elevator elevate, DoubleSupplier axis) {
    m_elevatorSubsystem = elevate;
    this.axis = axis;
    addRequirements(elevate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Power = axis.getAsDouble();
    Power = MathUtil.applyDeadband(Power, Constants.stickDeadband);
    if (Math.abs(Power) > 0) {
      m_elevatorSubsystem.setManual(-Power);
      locked = false;
    } else {
      if (!locked) {
        m_elevatorSubsystem.setManual(0);
        // m_elevatorSubsystem.stopHere();
        locked = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
