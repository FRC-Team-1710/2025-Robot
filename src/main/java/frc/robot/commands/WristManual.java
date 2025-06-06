// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.claw.Claw;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristManual extends Command {
  private Claw claw;
  private DoubleSupplier axis;
  boolean locked = true;
  private boolean enabled = false;

  /** Creates a new WristManual. */
  public WristManual(Claw claw, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.axis = power;
    this.claw = claw;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (Constants.simMode != Constants.Mode.SIM) {
    //   if (enabled != DriverStation.isEnabled()) {
    //     enabled = DriverStation.isEnabled();
    //     if (enabled) {
    //       if (claw.hasZeroed()) {
    //         claw.GOTOIDLE().schedule();
    //       } else {
    //         (new ZeroRizz(claw, axis)).schedule();
    //       }
    //     }
    //   }
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Power = axis.getAsDouble();
    Power = MathUtil.applyDeadband(Power, Constants.stickDeadband);
    if (Math.abs(Power) > 0) {
      claw.wristManual(Power * 2);
      locked = false;
    } else {
      if (!locked) {
        claw.wristManual(0);
        // claw.stopHere();
        locked = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    enabled = DriverStation.isEnabled();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
