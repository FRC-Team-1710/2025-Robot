// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeForAuto extends Command {
  private Manipulator m_Manipulator;
  private Funnel funnel;

  /** Creates a new IntakeCoral. */
  public IntakeForAuto(Manipulator manipulator, Funnel funnel) {
    this.m_Manipulator = manipulator;
    this.funnel = funnel;
    addRequirements(manipulator, funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Manipulator.runPercent(ManipulatorConstants.intakeSpeed);
    funnel.setRollerPower(Constants.FunnelConstants.FUNNEL_FAST);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (funnel.getFunnelStatorCurrent().lt(Amps.of(0))) {
    //   m_Manipulator.runPercent(-0.075);
    //   funnel.setRollerPower(-0.075);
    // } else
    // if (m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()
    //     || !m_Manipulator.beam1Broken() && !m_Manipulator.beam2Broken()) {
    //   m_Manipulator.runPercent(ManipulatorConstants.intakeSpeed);
    //   funnel.setRollerPower(Constants.FunnelConstants.FUNNEL_SLOW);
    // } else if (!m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()) {
    //   m_Manipulator.runPercent(0);
    //   funnel.setRollerPower(0);
    // }
    if (m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()) {
      m_Manipulator.runPercent(ManipulatorConstants.insideSpeed);
      funnel.setRollerPower(FunnelConstants.FUNNEL_SLOW);
    } else if (!m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken()) {
      m_Manipulator.runPercent(0);
      funnel.setRollerPower(0);
    } else if (!m_Manipulator.beam1Broken() && !m_Manipulator.beam2Broken()) {
      m_Manipulator.runPercent(ManipulatorConstants.intakeSpeed);
      funnel.setRollerPower(FunnelConstants.FUNNEL_FAST);
    } else if (m_Manipulator.beam1Broken() && !m_Manipulator.beam2Broken()) {
      m_Manipulator.runPercent(ManipulatorConstants.insideSpeed);
      funnel.setRollerPower(FunnelConstants.FUNNEL_SLOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Manipulator.runPercent(0);
    funnel.setRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!m_Manipulator.beam1Broken() && m_Manipulator.beam2Broken());
  }
}
