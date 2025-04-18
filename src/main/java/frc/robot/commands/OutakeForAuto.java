// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutakeForAuto extends Command {
  private Elevator elevator;
  private Manipulator manipulator;
  private Drive drivetrain;
  private RobotCentric requestsupplier;
  public final Timer timer = new Timer();

  /** Creates a new PlaceCoral. */
  public OutakeForAuto(
      Elevator elevator, Manipulator manipulator, Drive drivetrain, RobotCentric requestsupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.drivetrain = drivetrain;
    this.requestsupplier = requestsupplier;
    addRequirements(elevator, manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    drivetrain.stop(requestsupplier);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Logger.recordOutput("Command", "Outtake");
    if (elevator.isAtTarget()) {
      manipulator.runPercent(ManipulatorConstants.outtakeSpeed);
      drivetrain.stop(requestsupplier);
    } else if (timer.get() > 3) {
      drivetrain
          .applyRequest(() -> requestsupplier.withVelocityX(-0.3).withVelocityY(-0.3))
          .until(() -> timer.get() == 3.5);
    } else {
      elevator.setElevatorPosition(elevator.getMode());
      drivetrain.stop(requestsupplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.runPercent(0);
    drivetrain.stop(requestsupplier);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Constants.currentMode == Constants.Mode.SIM) return false;
    return ((!manipulator.beam1Broken() && !manipulator.beam2Broken()));
  }
}
