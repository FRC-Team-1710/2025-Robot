// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorConstants;
import frc.robot.subsystems.superstructure.manipulator.SimCoral;
import frc.robot.utils.TunableController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private Manipulator m_Manipulator;
  private Funnel funnel;
  private TunableController controller;
  private BooleanSupplier mechLB;
  private DoubleSupplier posex;
  private DoubleSupplier posey;
  private DoubleSupplier poserotation;

  /** Creates a new IntakeCoral. */
  public IntakeCoral(
      Manipulator manipulator,
      Funnel funnel,
      TunableController control,
      BooleanSupplier mechLB,
      DoubleSupplier posex,
      DoubleSupplier posey,
      DoubleSupplier poserotation) {
    this.m_Manipulator = manipulator;
    this.funnel = funnel;
    this.controller = control;
    this.mechLB = mechLB;
    this.posex = posex;
    this.posey = posey;
    this.poserotation = poserotation;
    addRequirements(manipulator, funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SimCoral.intake(posex, posey, poserotation).schedule();
    m_Manipulator.runPercent(ManipulatorConstants.intakeSpeed);
    funnel.setRollerPower(FunnelConstants.FUNNEL_FAST);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mechLB.getAsBoolean()) {
      if (m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()) {
        m_Manipulator.runPercent(ManipulatorConstants.insideSpeed);
        controller.setRumble(RumbleType.kBothRumble, 0);
        funnel.setRollerPower(FunnelConstants.FUNNEL_SLOW);
      } else if (!m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()) {
        m_Manipulator.runPercent(0);
        funnel.setRollerPower(0);
        controller.setRumble(RumbleType.kBothRumble, 1);
      } else if (!m_Manipulator.beam1Broken() && !m_Manipulator.beam2Broken()) {
        m_Manipulator.runPercent(ManipulatorConstants.intakeSpeed);
        funnel.setRollerPower(FunnelConstants.FUNNEL_FAST);
        controller.setRumble(RumbleType.kBothRumble, 0);
      } else if (m_Manipulator.beam1Broken() && !m_Manipulator.beam2Broken()) {
        m_Manipulator.runPercent(ManipulatorConstants.insideSpeed);
        controller.setRumble(RumbleType.kBothRumble, 0);
        funnel.setRollerPower(FunnelConstants.FUNNEL_SLOW);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!mechLB.getAsBoolean()) {
      m_Manipulator.runPercent(0);
      funnel.setRollerPower(0);
      controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
