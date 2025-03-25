// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.Conversions;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorEncoder {
  private final EncoderType encoderType;

  private Distance encoderoffset = Inches.of(0);

  private final CANcoder cancoder;
  private final CANrange canrange;
  private final LaserCan lasercan;

  private final TalonFX motor;

  public enum EncoderType {
    stringEncoder(),
    canRange(),
    motorEncoders(),
    lasercan()
  }

  /** ID: 13 */
  public ElevatorEncoder(EncoderType encoderType, TalonFX motor) {
    this.motor = motor;
    this.encoderType = encoderType;
    // switch (encoderType) {
    //   case lasercan:
    cancoder = null;
    canrange = null;
    CanBridge.runTCP();
    lasercan = new LaserCan(22);
    try {
      lasercan.setRangingMode(RangingMode.LONG);
    } catch (ConfigurationFailedException error) {
      Logger.recordOutput(
          "Laser Can Error",
          "Error "
              + error.getErrorCode()
              + ": Bruh, the lasercan didn't set the ranging mode. :skull: "
              + error.getMessage());
    }
    encoderoffset = Inches.of(0); // Change
    // break;
    // case stringEncoder:
    //   cancoder = new CANcoder(13);
    //   cancoder.optimizeBusUtilization(4, 0.1);
    //   cancoder.setPosition(0);
    //   canrange = null;
    //   lasercan = null;
    //   break;
    // case canRange:
    //   cancoder = null;
    //   canrange = new CANrange(13);
    //   canrange.optimizeBusUtilization(4, 0.1);
    //   encoderoffset = Inches.of(0); // Change
    //   lasercan = null;
    //   break;
    // default:
    //   cancoder = null;
    //   canrange = null;
    //   lasercan = null;

  }

  public Distance getLasercanDistance() {
    // if (isLasercan()) {
    Measurement measurment = lasercan.getMeasurement();
    if (measurment != null) {
      Logger.recordOutput("Laser Can Error", "No errors getting distance yet :)");
      return Inches.of(
          Units.metersToInches(Double.valueOf(lasercan.getMeasurement().distance_mm) / 1000)
              - encoderoffset.in(Meters));
    }
    Logger.recordOutput(
        "Laser Can Error",
        "Error: measurment is literally null. Why can't the string encoder just work?");
    return null;
    // }
    // return Inches.of(1);
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

  public boolean isLasercan() {
    if (encoderType == EncoderType.lasercan) {
      return true;
    }
    return false;
  }
}
