// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.Conversions;

/** Add your docs here. */
public class ElevatorEncoder {
  private final EncoderType encoderType;

  private Distance encoderoffset = Inches.of(0);

  private final CANcoder cancoder;
  private final CANrange canrange;

  private final TalonFX motor;

  public enum EncoderType {
    stringEncoder(),
    canRange(),
    motorEncoders(),
  }

  /** ID: 13 */
  public ElevatorEncoder(EncoderType encoderType, TalonFX motor) {
    this.motor = motor;
    this.encoderType = encoderType;
    switch (encoderType) {
      case stringEncoder:
        cancoder = new CANcoder(13);
        cancoder.optimizeBusUtilization(4, 0.1);
        cancoder.setPosition(0);
        canrange = null;
        break;
      case canRange:
        cancoder = null;
        canrange = new CANrange(13);
        canrange.optimizeBusUtilization(4, 0.1);
        encoderoffset = Inches.of(0); // Change
        break;
      default:
        cancoder = null;
        canrange = null;
    }
  }

  public Distance getDistance() {
    switch (encoderType) {
      case stringEncoder:
        return Conversions.rotationsToDistance(
            cancoder.getPosition().getValue(), 1, Inches.of(0.7638888888888888));
      case canRange:
        return canrange.getDistance().getValue().minus(encoderoffset);
      default:
        return Conversions.rotationsToDistance(
            motor.getPosition().getValue(), 6, Inches.of(1.1338619402985));
    }
  }

  public AngularVelocity getSpeed() {
    switch (encoderType) {
      case stringEncoder:
        return cancoder.getVelocity().getValue();
      default:
        return motor.getVelocity().getValue();
    }
  }

  public void zero() {
    switch (encoderType) {
      case stringEncoder:
        cancoder.setPosition(0);
        break;
      default:
    }
  }

  public boolean isCancoder() {
    if (encoderType == EncoderType.stringEncoder) {
      return true;
    }
    return false;
  }

  public boolean isCanrange() {
    if (encoderType == EncoderType.canRange) {
      return true;
    }
    return false;
  }

  public boolean isMotorEncoders() {
    if (encoderType == EncoderType.motorEncoders) {
      return true;
    }
    return false;
  }
}
