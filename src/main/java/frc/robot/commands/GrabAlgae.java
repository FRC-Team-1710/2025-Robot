// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.claw.Claw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabAlgae extends Command {
  Claw claw;
  Timer timer = new Timer();

  /** Creates a new GrabAlgae. */
  public GrabAlgae(Claw claw) {
    this.claw = claw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setRollers(.5);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.lockRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Constants.currentMode == Mode.SIM && timer.get() > .25) {
    //   claw.setAlgaeStatus(true);
    //   return true;
    // }
    if (timer.get() > .25 && claw.getRollerCurrent() < -60 && Constants.currentMode != Mode.SIM) {
      claw.setAlgaeStatus(true);
      claw.setRollerPositionWhenAlgaeGrabbed(claw.getRollerPosition());
      return true;
    }
    return false;
  }
}
