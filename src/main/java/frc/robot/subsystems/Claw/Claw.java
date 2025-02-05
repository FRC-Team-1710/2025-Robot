// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private TalonFX clawRollers;
  private TalonFX wrist;
  private CANcoder ClawEncoder;

  private double CanEncoderOffset = 0;
  private double setPoint = 0;
  private double ClawGearing = 9 * (48 / 18);

  // PID
  private PIDController PIDWrist;
  private double VelocityP = 0;
  private double VelocityI = 0;
  private double VelocityD = 0;

  public Claw() {
    clawRollers = new TalonFX(0);
    wrist = new TalonFX(0);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    clawRollers.getConfigurator().apply(config);
    wrist.getConfigurator().apply(config);

    PIDWrist = new PIDController(VelocityP, VelocityI, VelocityD);

    wrist.setPosition(wrist.getPosition().getValueAsDouble() - CanEncoderOffset);
  }

  public void setClawPower(double power) {
    clawRollers.set(power);
  }

  public double getPosition() {
    return clawRollers.getPosition().getValueAsDouble() / ClawGearing;
  }

  public void setWrist(double position) {
    setPoint = position * ClawGearing;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wrist.set(PIDWrist.calculate(wrist.getPosition().getValueAsDouble(), setPoint));
  }
}
