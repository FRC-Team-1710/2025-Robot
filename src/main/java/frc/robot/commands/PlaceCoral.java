// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorConstants;
import frc.robot.subsystems.superstructure.manipulator.SimCoral;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TunableController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlaceCoral extends Command {
  private Elevator elevator;
  private Manipulator manipulator;
  private TunableController controller;

  /** Creates a new PlaceCoral. */
  public PlaceCoral(Elevator elevator, Manipulator manipulator, TunableController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.controller = controller;
    addRequirements(elevator, manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.isAtTarget()
        && elevator.getMode() == TargetingComputer.getCurrentTargetLevel().getElevatorPosition()) {
      manipulator.runPercent(ManipulatorConstants.outtakeSpeed);
      if (!manipulator.beam1Broken() && !manipulator.beam2Broken()) {
        controller.setRumble(RumbleType.kBothRumble, 1);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SimCoral.placeCoral();
    manipulator.runPercent(0);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
