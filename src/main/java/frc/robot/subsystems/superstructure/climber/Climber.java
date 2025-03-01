// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climber;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;

public class Climber extends SubsystemBase {
  private TalonFX climber; // Right
  public boolean goForClimb;

  public Timer timer = new Timer();

  public Orchestra m_orchestra = new Orchestra();

  public Climber() {
    climber = new TalonFX(41);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Audio.AllowMusicDurDisable = true;
    climber.getConfigurator().apply(config);

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

    m_orchestra.addInstrument(climber, 0);

    m_orchestra.play();
    timer.reset();
    timer.start();
  }

  public void SetClimberPower(double power) {
    climber.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
