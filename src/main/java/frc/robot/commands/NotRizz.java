// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NotRizz extends Command {
  Elevator elevator;
  Timer timer = new Timer();

  /** Creates a new GrabAlgae. */
  public NotRizz(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("zerod?", false);
    elevator.setManual(-0.1);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("zerod?", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Constants.currentMode == Mode.SIM && timer.get() > .25) {
      return true;
    }
    if (timer.get() > .25
        && elevator.getElevatorCurrent() > 55
        && Constants.currentMode != Mode.SIM) {
      elevator.setManual(0);
      elevator.zero();
      return true;
    }
    return false;
  }
}
