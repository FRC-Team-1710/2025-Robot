// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndIntake extends Command {
  private Manipulator manipulator;
  private Funnel funnel;
  public final Timer timer = new Timer();

  /** Creates a new EndIntake. */
  public EndIntake(Manipulator manipulator, Funnel funnel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.funnel = funnel;
    this.manipulator = manipulator;
    addRequirements(manipulator, funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((manipulator.beam1Broken() && !manipulator.beam2Broken())
        || (manipulator.beam1Broken() && manipulator.beam2Broken())) {
      timer.reset();
      timer.stop();
      manipulator.runPercent(ManipulatorConstants.insideSpeed);
      funnel.SetRollerPower(.2);
    } else if (!manipulator.beam1Broken() && !manipulator.beam2Broken()) {
      if (!timer.isRunning()) {
        timer.start();
      }
      manipulator.runPercent(ManipulatorConstants.insideSpeed);
      funnel.SetRollerPower(.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.runPercent(0);
    funnel.SetRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((!manipulator.beam1Broken() && manipulator.beam2Broken()) || timer.get() > 3) {
      return true;
    } else {
      return false;
    }
  }
}
