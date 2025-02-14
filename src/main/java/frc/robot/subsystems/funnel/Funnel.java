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

  public void SetRollerPower(double power) {
    RollerLeader.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
