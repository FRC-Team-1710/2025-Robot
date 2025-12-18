// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;

@Logged
public class Climber {
  @Logged(name = "Motor", importance = Importance.CRITICAL)
  private TalonFX climber;

  @Logged(name = "GoForClimb", importance = Importance.CRITICAL)
  public boolean goForClimb;

  @Logged(name = "SafeToRetract", importance = Importance.INFO)
  public boolean safeToRetract = false;

  @Logged(name = "CurrentState", importance = Importance.CRITICAL)
  private ClimberStates currentState = ClimberStates.STOWED;

  @NotLogged private double gearRatio = 80;

  public Climber() {
    climber = new TalonFX(41);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Audio.AllowMusicDurDisable = true;
    climber.getConfigurator().apply(config);

    climber.setPosition(0);

    // if (Constants.useSmartDashboard) {
    //   SmartDashboard.putBoolean("safe to retract", safeToRetract);
    // }
  }

  public void SetClimberPower(double power) {
    climber.set(power);
  }

  /* Degrees */
  @Logged(name = "Position", importance = Importance.CRITICAL)
  public double getPosition() {
    return climber.getPosition().getValueAsDouble() / gearRatio;
  }

  public void periodic() {
    // if (Constants.useSmartDashboard) {
    //   SmartDashboard.putNumber("Climber Position", getPosition());
    //   if (SmartDashboard.getBoolean("safe to retract", safeToRetract) != safeToRetract) {
    //     safeToRetract = SmartDashboard.getBoolean("safe to retract", safeToRetract);
    //   }
    // }

    switch (currentState) {
      case STOWED:
        climber.stopMotor();
        break;
      case OUT:
        if (getPosition() > 2.1) {
          climber.stopMotor();
        } else {
          climber.set(0.8);
        }
        break;
      case CLIMBED:
        if (getPosition() > 3.9) {
          climber.stopMotor();
        } else {
          climber.set(0.4);
        }
        break;
      default:
        break;
    }
  }

  @Logged(name = "HasClimbed", importance = Importance.CRITICAL)
  public boolean hasClimbed() {
    return getPosition() > 3.9;
  }

  public enum ClimberStates {
    STOWED(),
    OUT(),
    CLIMBED(),
    MANUAL()
  }

  public void setState(ClimberStates state) {
    if (state == ClimberStates.MANUAL) {
      currentState = ClimberStates.MANUAL;
    } else if (currentState == ClimberStates.STOWED && state == ClimberStates.OUT) {
      this.currentState = state;
    } else if (currentState == ClimberStates.OUT && state == ClimberStates.CLIMBED) {
      this.currentState = state;
    }
  }

  public void setState(double power) {
    if (power >= 0 || safeToRetract) {
      climber.set(power);
    }
  }
}
