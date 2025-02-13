// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.TargetingComputer.Targets;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignmentForAuto extends Command {
  Vision vision;
  Targets target;
  Drive drivetrain;
  SwerveRequest robotCentric;
  double current;
  CommonTalon drivemotor = TunerConstants.createDrivetrain().getModule(2).getDriveMotor();
  /** Creates a new MyMentalHealth. */
  public AlignmentForAuto(Vision vision, Drive drivetrain, Targets target) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.target = target;

    addRequirements(vision, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotCentric =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(
                TunerConstants.kSpeedAt12Volts.times(
                    -(target.getOffset().getX()
                            - vision.calculateOffset(target.getApriltag(), target.getOffset()).getX())
                        * 0.25))
            .withVelocityY(
                TunerConstants.kSpeedAt12Volts.times(
                    -(target.getOffset().getY()
                            - vision.calculateOffset(target.getApriltag(), target.getOffset()).getY())
                        * 0.25))
            .withRotationalRate(
                Constants.MaxAngularRate.times(
                    (new Rotation2d(Units.degreesToRadians(target.getTargetingAngle()))
                            .minus(drivetrain.getPose().getRotation())
                            .getRadians())
                        * 0.25));

    drivetrain.setControl(robotCentric);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drivetrain.hasSpeed(vision);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return target.getOffset().getX()
    - vision.calculateOffset(target.getApriltag(), target.getOffset()).getX()
< 0.01;
  }
}
