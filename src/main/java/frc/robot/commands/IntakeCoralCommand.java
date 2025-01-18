// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.utils.TunableController;
import java.lang.ModuleLayer.Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralCommand extends Command {
  private CoralIntake CoralIntake;
  private TunableController Controller;

  /** Creates a new IntakeCoralCommand. */
  public IntakeCoralCommand(CoralIntake m_coralIntake, TunableController joystick) {
    CoralIntake = m_coralIntake;
    this.Controller = joystick;
    addRequirements(CoralIntake);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Claw/Coral Intaking?", true);
    CoralIntake.runPercent(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (CoralIntake.beam1Broken()) {
      // If coral is intaked
      CoralIntake.runPercent(0);
      Controller.setRumble(RumbleType.kBothRumble, 0.75);
      SmartDashboard.putBoolean("Claw/Coral Intaking?", false);
    } else {
      // If the coral hasn't been intaked
      Controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Controller.setRumble(RumbleType.kBothRumble, 0);
    SmartDashboard.putBoolean("Claw/Coral Intaking?", false);
    CoralIntake.runPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
