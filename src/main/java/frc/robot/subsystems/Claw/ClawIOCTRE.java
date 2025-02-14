// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawIOCTRE implements ClawIO {
  public static final double GEAR_RATIO = 1;
  private boolean locked = false;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  public final TalonFX wrist = new TalonFX(21);
  public final TalonFX intake = new TalonFX(22);

  private final PIDController wristPID = new PIDController(kP, kI, kD);

  private final StatusSignal<Angle> wristPosition = wrist.getPosition();
  private final StatusSignal<AngularVelocity> wristVelocity = wrist.getVelocity();
  private final StatusSignal<Voltage> wristAppliedVolts = wrist.getMotorVoltage();
  private final StatusSignal<Current> wristStatorCurrent = wrist.getStatorCurrent();
  private final StatusSignal<Current> wristSupplyCurrent = wrist.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> intakeVelocity = intake.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = intake.getMotorVoltage();
  private final StatusSignal<Current> intakeStatorCurrent = intake.getStatorCurrent();
  private final StatusSignal<Current> intakeSupplyCurrent = intake.getSupplyCurrent();

  private Angle SetAngle = Rotations.of(0);
  private double wristManual = 0.0;
  private double runPercent = 0.0;

  public ClawIOCTRE() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wrist.getConfigurator().apply(config);
    intake.getConfigurator().apply(config);

    SmartDashboard.putNumber("Claw/PID/P", kP);
    SmartDashboard.putNumber("Claw/PID/I", kI);
    SmartDashboard.putNumber("Claw/PID/D", kD);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristPosition,
        wristVelocity,
        wristStatorCurrent,
        wristSupplyCurrent,
        intakeVelocity,
        intakeStatorCurrent,
        intakeSupplyCurrent);
  }

  @Override
  public void updateInputs(ClawIOInputsAutoLogged inputs) {
    var leaderStatus =
        BaseStatusSignal.refreshAll(
            wristPosition,
            wristVelocity,
            wristAppliedVolts,
            wristStatorCurrent,
            wristSupplyCurrent);

    var followerStatus =
        BaseStatusSignal.refreshAll(
            intakeVelocity, intakeAppliedVolts, intakeStatorCurrent, intakeSupplyCurrent);

    inputs.wristMotorRotations = wristPosition.getValue().div(GEAR_RATIO);

    inputs.wristVelocity = wristVelocity.getValue().div(GEAR_RATIO);
    inputs.intakeVelocity = intakeVelocity.getValue().div(GEAR_RATIO);

    inputs.wristAppliedVoltage = wristAppliedVolts.getValue();
    inputs.wristStatorCurrent = wristStatorCurrent.getValue();
    inputs.wristSupplyCurrent = wristSupplyCurrent.getValue();
    inputs.intakeAppliedVoltage = intakeAppliedVolts.getValue();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValue();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValue();

    inputs.setpoint = SetAngle;
    inputs.wristManual = wristManual;
    inputs.intakePercent = runPercent;

    inputs.angle = Degrees.of((wristPosition.getValueAsDouble() / GEAR_RATIO));

    SmartDashboard.putNumber("Claw/Motors/Wrist/Position", wristPosition.getValue().magnitude());
    SmartDashboard.putNumber("Claw/Motors/Wrist/Velocity", wristVelocity.getValue().magnitude());
    SmartDashboard.putNumber("Claw/Motors/Intake/Velocity", intakeVelocity.getValue().magnitude());
    SmartDashboard.putNumber(
        "Claw/Motors/Wrist/Applied Volt", wristAppliedVolts.getValue().magnitude());
    SmartDashboard.putNumber(
        "Claw/Motors/Wrist/Stator Current", wristStatorCurrent.getValue().magnitude());
    SmartDashboard.putNumber(
        "Claw/Motors/Wrist/Supply Current", wristSupplyCurrent.getValue().magnitude());
    SmartDashboard.putNumber(
        "Claw/Motors/Intake/Applied Volt", intakeAppliedVolts.getValue().magnitude());
    SmartDashboard.putNumber(
        "Claw/Motors/Intake/Stator Current", intakeStatorCurrent.getValue().magnitude());
    SmartDashboard.putNumber(
        "Claw/Motors/Intake/Supply Current", intakeSupplyCurrent.getValue().magnitude());
  }

  @Override
  public void updatePID(ClawIOInputsAutoLogged inputs) {
    tempPIDTuning();
    if (locked) {
      wrist.setVoltage(wristPID.calculate(inputs.angle.magnitude()));
    }
    SmartDashboard.putNumber("Claw/Angle DEGREES", inputs.angle.magnitude());
    SmartDashboard.putNumber("Claw/Setpoint", wristPID.getSetpoint());
  }

  @Override
  public void setAngle(Angle angle) {
    SetAngle = angle;
    locked = true;
  }

  @Override
  public void setManual(double power) {
    locked = false;
    wristManual = power;
    wrist.set(wristManual);
  }

  @Override
  public void runPercent(double power) {
    runPercent = power;
    intake.set(power);
  }

  private void tempPIDTuning() {
    if (kP != SmartDashboard.getNumber("Claw/PID/P", kP)) {
      kP = SmartDashboard.getNumber("Claw/PID/P", kP);
      wristPID.setP(kP);
    }

    if (kI != SmartDashboard.getNumber("Claw/PID/I", kI)) {
      kI = SmartDashboard.getNumber("Claw/PID/I", kI);
      wristPID.setI(kI);
    }

    if (kD != SmartDashboard.getNumber("Claw/PID/D", kD)) {
      kD = SmartDashboard.getNumber("Claw/PID/D", kD);
      wristPID.setD(kD);
    }
  }
}
