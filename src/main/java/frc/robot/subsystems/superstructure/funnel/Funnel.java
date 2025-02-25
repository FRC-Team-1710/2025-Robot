// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.funnel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  private TalonFX RollerLeader; // Right
  private TalonFX RollerFollower;
  private TalonFX Pivot;
  private static boolean hasCoral;

  public Funnel() {
    RollerLeader = new TalonFX(31);
    RollerFollower = new TalonFX(32);
    Pivot = new TalonFX(30);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RollerLeader.getConfigurator().apply(config);
    RollerFollower.getConfigurator().apply(config);

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    Pivot.getConfigurator().apply(pivotConfig);

    RollerFollower.setControl(new Follower(RollerLeader.getDeviceID(), true));
  }

  public void setRollerPower(double power) {
    RollerLeader.set(power);
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
