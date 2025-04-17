// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.claw.Claw;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroRizz extends Command {
  Claw claw;
  Timer timer = new Timer();
  DoubleSupplier mechrs;

  /** Creates a new GrabAlgae. */
  public ZeroRizz(Claw claw, DoubleSupplier mechrs) {
    this.claw = claw;
    this.mechrs = mechrs;
    addRequirements(claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("zero?", false);
    if (Constants.currentMode != Mode.SIM) {
      claw.wristManual(-0.5); // Volts
    }
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("zero?", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Constants.currentMode == Mode.SIM && timer.get() > .25) {
      claw.IDLE().schedule();
      return true;
    }
    if (Math.abs(MathUtil.applyDeadband(mechrs.getAsDouble(), Constants.stickDeadband)) > 0) {
      return true;
    }
    if ((timer.get() > .25 && claw.getWristCurrent() > 18 && Constants.currentMode != Mode.SIM)) {
      claw.wristManual(0);
      claw.zero();
      return true;
    }
    return false;
  }
}
