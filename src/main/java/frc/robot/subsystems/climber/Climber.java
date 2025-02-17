// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private TalonFX climber; // Right

  private States state = States.Off;

  public Climber() {
    climber = new TalonFX(41);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climber.getConfigurator().apply(config);
  }

  public void setClimberPower(double power) {
    climber.set(power);
  }

  public enum States {
    Up(0.25),
    Down(0.25),
    Off;

    private final double percent;

    States() {
      this.percent = 0.0;
    }

    States(double percent) {
      this.percent = percent;
    }
  }

  @Override
  public void periodic() {
    setClimberPower(state.percent);
  }

  public void Off() {
    this.state = States.Off;
  }

  public void Up() {
    this.state = States.Up;
  }

  public void Down() {
    this.state = States.Down;
  }
}
