// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.Claw.ClawStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TossAlgae extends Command {
  Claw claw;
  Timer timer = new Timer();

  /** Creates a new TossAlgae. */
  public TossAlgae(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.TOSS().schedule();
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (claw.isAtTarget() && claw.getMode() == ClawStates.TOSS) {
      claw.RELEASE().schedule();
      timer.restart();
    }
    if (timer.get() > .3) {
      claw.setRollers(-.2);
      if (Constants.simMode == Mode.SIM) claw.setAlgaeStatus(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.IDLE();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
