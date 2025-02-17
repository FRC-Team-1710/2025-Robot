// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  private TalonFX RollerLeader; // Right
  private TalonFX RollerFollower;

  private RollerStates rollerState = RollerStates.Off;

  public Funnel() {
    RollerLeader = new TalonFX(31);
    RollerFollower = new TalonFX(32);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RollerLeader.getConfigurator().apply(config);
    RollerFollower.getConfigurator().apply(config);

    RollerFollower.setControl(new Follower(RollerLeader.getDeviceID(), true));
  }

  public enum AngleStates {
    Up(0),
    Down(0);

    private final double angle;

    AngleStates(double angle) {
      this.angle = angle;
    }
  }

  public enum RollerStates {
    Intake(0.5),
    Inside(0.25),
    Off;

    private final double percent;

    RollerStates() {
      this.percent = 0.0;
    }

    RollerStates(double percent) {
      this.percent = percent;
    }
  }

  private void setRollerPower(double power) {
    RollerLeader.set(power);
  }

  @Override
  public void periodic() {
    setRollerPower(rollerState.percent);
  }

  public void Off() {
    this.rollerState = RollerStates.Off;
  }

  public void Intake() {
    this.rollerState = RollerStates.Intake;
  }

  public void Inside() {
    this.rollerState = RollerStates.Inside;
  }
}
