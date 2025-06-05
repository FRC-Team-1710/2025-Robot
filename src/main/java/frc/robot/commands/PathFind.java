// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.TargetingComputer;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFind extends Command {
  PathConstraints constraints =
      new PathConstraints(4.5, 1.5, Units.degreesToRadians(467), Units.degreesToRadians(500));
  GoalEndState end =
      new GoalEndState(
          MetersPerSecond.of(0.0), TargetingComputer.getCurrentTargetBranchPose().getRotation());
  Command currentcommand;
  boolean nullPath = false;

  /** Creates a new PathFind. */
  public PathFind(Drive drivetrain) {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nullPath = false;
    end =
        new GoalEndState(
            MetersPerSecond.of(0.0), TargetingComputer.getCurrentTargetBranchPose().getRotation());
    if (Pathfinding.getCurrentPath(constraints, end) != null) {
      Logger.recordOutput(
          "Pathfinding Path",
          Pathfinding.getCurrentPath(constraints, end)
              .getPathPoses()
              .toArray(
                  new Pose2d[Pathfinding.getCurrentPath(constraints, end).getPathPoses().size()]));
      currentcommand = AutoBuilder.followPath(Pathfinding.getCurrentPath(constraints, end));
      currentcommand.schedule();
    } else {
      nullPath = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentcommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return nullPath; // ? true : currentcommand.isFinished();
  }
}
