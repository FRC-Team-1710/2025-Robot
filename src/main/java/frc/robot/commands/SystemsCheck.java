// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.utils.TunableController;

/** Command to run through each mechanism on the robot. */
public class SystemsCheck extends Command {
  // Boolean to check for command completion
  boolean checkComplete;

  // Verification of test mode
  boolean isTestMode;

  // Temporary controller
  TunableController testController;
  Trigger incrementStep = new Trigger(testController.a());

  // Subsystems
  Drive driveTrain;
  Claw clawSubsystem;
  Climber climberSubsystem;
  Elevator elevatorSubsystem;
  Funnel funnelSubsystem;
  Manipulator manipulatorSubsystem;

  // Drive request variables
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  SwerveRequest.RobotCentric robotCentric =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed.times(0.025))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.025))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Step of the systems check process
  int step;


  /** Creates a new SystemsCheck. */
  public SystemsCheck(
    TunableController testController,
    Drive driveTrain,
    Claw clawSubsystem,
    Climber climberSubsystem,
    Elevator elevatorSubsystem,
    Funnel funnelSubsystem,
    Manipulator manipulatorSubsystem) {

    // Test mode verification
    this.isTestMode = SmartDashboard.getBoolean("isTestMode", false);

    // Subsystems
    this.driveTrain = driveTrain;
    this.clawSubsystem = clawSubsystem;
    this.climberSubsystem = climberSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    this.manipulatorSubsystem = manipulatorSubsystem;

    // Defaults
    checkComplete = false;
    step = 0;
  }

  @Override
  public void initialize() {
    if (!isTestMode) { // If the robot is not in test mode, immediately exits the program
      // if this passes, WARNING LEAVE THE AREA IMMEDIATELY
      checkComplete = true;
    }
  }

  @Override
  public void execute() {
    // Record the current step of the systems check process
    Logger.recordOutput("Systems Check Step", step);

    if (incrementStep.getAsBoolean()) { // Once A is pressed, commence the next step
      initiateNextCheck();
      while (incrementStep.getAsBoolean()) {} // lowkey might crash the code...
    }

    if (step == 19) {
      checkComplete = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return checkComplete;
  }

  /**
   * Runs through the systems check one step at a time. Each run increments the step.
   */
  private void initiateNextCheck() {
    switch (step) {
      case 0: driveTrain.applyRequest(() -> robotCentric.withVelocityX(MaxSpeed.times(0.5))); break;
      case 1: driveTrain.applyRequest(() -> robotCentric.withVelocityX(MaxSpeed.times(-0.5))); break;
      case 2: driveTrain.applyRequest(() -> robotCentric.withRotationalRate(Constants.MaxAngularRate.times(0.5))); break;
      case 3: driveTrain.applyRequest(() -> robotCentric.withRotationalRate(Constants.MaxAngularRate.times(-0.5))); break;
      case 4: clawSubsystem.setRollers(0.5); break;
      case 5: clawSubsystem.setRollers(-0.5); break;
      case 6: clawSubsystem.setRollers(0); clawSubsystem.PROCESSOR(); break;
      case 7: clawSubsystem.IDLE(); break;
      case 8: break; // Engage the climb
      case 9: elevatorSubsystem.L4(); break;
      case 10: elevatorSubsystem.L2(); break;
      case 11: elevatorSubsystem.L3(); break;
      case 12: elevatorSubsystem.L1(); break;
      case 13: elevatorSubsystem.intake(); break;
      case 14: funnelSubsystem.setRollerPower(0.5); break;
      case 15: funnelSubsystem.setRollerPower(0); break; // Engage Funnel
      case 16: break; // Tuck Funnel
      case 17: manipulatorSubsystem.runPercent(0.5);
      case 18: manipulatorSubsystem.runPercent(0);
      default: break;
    }

    step++;
  }
}
