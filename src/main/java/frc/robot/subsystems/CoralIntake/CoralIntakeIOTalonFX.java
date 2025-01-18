// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.CoralIntake;

import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/**
 * This CoralIntake implementation is for a Talon FX driving a motor like the Falon 500 or Kraken
 * X60.
 */
public class CoralIntakeIOTalonFX implements CoralIntakeIO {
  private final TalonFX CoralIntake = new TalonFX(0);
  private final DigitalInput BreakingBeam1 = new DigitalInput(0);
  private final DigitalInput BreakingBeam2 = new DigitalInput(0);
  final StatusSignal<Angle> positionRot = CoralIntake.getPosition();
  final StatusSignal<AngularVelocity> velocityRotPerSec = CoralIntake.getVelocity();
  final StatusSignal<Voltage> appliedVolts = CoralIntake.getMotorVoltage();
  final StatusSignal<Current> currentAmps = CoralIntake.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public CoralIntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    // config.CurrentLimits.SupplyCurrentLimit = ;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> CoralIntake.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    CoralIntake.optimizeBusUtilization();
  }

  public void updateInputs(CoralIntakeIOInputs CoralIntakeIOInputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    CoralIntakeIOInputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    CoralIntakeIOInputs.velocityRadPerSec =
        Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    CoralIntakeIOInputs.appliedVolts = appliedVolts.getValueAsDouble();
    CoralIntakeIOInputs.currentAmps = currentAmps.getValueAsDouble();
    CoralIntakeIOInputs.beam1Broken = !BreakingBeam1.get();
    CoralIntakeIOInputs.beam2Broken = !BreakingBeam2.get();
  }

  @Override
  public void setVoltage(double volts) {
    CoralIntake.setControl(voltageRequest.withOutput(volts));
  }
}
