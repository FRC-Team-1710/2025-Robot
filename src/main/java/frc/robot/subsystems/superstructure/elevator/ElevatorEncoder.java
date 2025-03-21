// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.Conversions;

/** Add your docs here. */
public class ElevatorEncoder {
  private final EncoderType encoderType;

  private Distance canrangeoffset = Inches.of(0);

  private final CANcoder cancoder;
  private final CANrange canrange;

  public enum EncoderType {
    stringEncoder(),
    canRange(),
    motorEncoders()
  }

  /** ID: 13 */
  public ElevatorEncoder(EncoderType encoderType) {
    this.encoderType = encoderType;
    switch (this.encoderType) {
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
        canrangeoffset = canrange.getDistance().getValue();
        break;
      default:
        cancoder = null;
        canrange = null;
    }
  }

  public Distance getDistance() {
    switch (encoderType) {
      case stringEncoder:
        return Conversions.rotationsToDistance(cancoder.getPosition().getValue(), 1, Inches.of(0.7638888888888888));
      case canRange:
        return canrange.getDistance().getValue().minus(canrangeoffset);
      default:
        return null;
    }
  }

  public AngularVelocity getSpeed() {
    switch (encoderType) {
      case stringEncoder:
        return cancoder.getVelocity().getValue();
      case canRange:
        return DegreesPerSecond.of(0);
      default:
        return null;
    }
  }

  public void zero() {
    switch (encoderType) {
      case stringEncoder:
        cancoder.setPosition(0);
      case canRange:
        canrangeoffset = canrange.getDistance().getValue();
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
