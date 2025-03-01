// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.funnel;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;

public class Funnel extends SubsystemBase {
  private TalonFX RollerLeader; // Right
  private TalonFX RollerFollower;
  private TalonFX Pivot;
  private static boolean hasCoral;

  public Timer timer = new Timer();

  public Orchestra m_orchestra = new Orchestra();

  public Funnel() {
    RollerLeader = new TalonFX(31);
    RollerFollower = new TalonFX(32);
    Pivot = new TalonFX(30);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Audio.AllowMusicDurDisable = true;
    RollerLeader.getConfigurator().apply(config);
    RollerFollower.getConfigurator().apply(config);

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfig.Audio.AllowMusicDurDisable = true;
    Pivot.getConfigurator().apply(pivotConfig);

    RollerFollower.setControl(new Follower(RollerLeader.getDeviceID(), true));

    // Attempt to load the chrp
    var status =
        m_orchestra.loadMusic(
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("orchestra" + File.separator + "dangerzone.chrp")
                .toString());

    if (!status.isOK()) {
      // log error
    }

    m_orchestra.addInstrument(RollerLeader, 0);
    m_orchestra.addInstrument(RollerFollower, 1);
    m_orchestra.addInstrument(Pivot, 2);

    m_orchestra.play();
    timer.reset();
    timer.start();
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
    if (timer.get() > 15) {
      if (m_orchestra.isPlaying()) {
        m_orchestra.stop();
      }
      m_orchestra.close();
      timer.stop();
      timer.reset();
    }
  }
}
