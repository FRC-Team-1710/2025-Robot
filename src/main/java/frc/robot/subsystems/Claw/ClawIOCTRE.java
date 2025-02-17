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
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kacel = 0.0;
  private double kvel = 0.0;

  public final TalonFX wrist = new TalonFX(21);
  public final TalonFX intake = new TalonFX(22);

  private final ProfiledPIDController wristPID =
      new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kvel, kacel));
  private final ArmFeedforward wristFF = new ArmFeedforward(kS, kG, kV, kA);

  private final StatusSignal<Angle> wristPosition = wrist.getPosition();
  private final StatusSignal<AngularVelocity> wristVelocity = wrist.getVelocity();
  private final StatusSignal<Voltage> wristAppliedVolts = wrist.getMotorVoltage();
  private final StatusSignal<Current> wristStatorCurrent = wrist.getStatorCurrent();
  private final StatusSignal<Current> wristSupplyCurrent = wrist.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> intakeVelocity = intake.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = intake.getMotorVoltage();
  private final StatusSignal<Current> intakeStatorCurrent = intake.getStatorCurrent();
  private final StatusSignal<Current> intakeSupplyCurrent = intake.getSupplyCurrent();

  private final Debouncer clawDebounce = new Debouncer(0.5);
  private final Debouncer wristDebounce = new Debouncer(0.5);

  private Angle SetAngle = Degrees.of(0);
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
  public void updateInputs(ClawIOInputs inputs) {
    StatusCode wristStatus =
        BaseStatusSignal.refreshAll(
            wristPosition,
            wristVelocity,
            wristAppliedVolts,
            wristStatorCurrent,
            wristSupplyCurrent);

    StatusCode clawStatus =
        BaseStatusSignal.refreshAll(
            intakeVelocity, intakeAppliedVolts, intakeStatorCurrent, intakeSupplyCurrent);

    inputs.clawConnected = clawDebounce.calculate(clawStatus.isOK());
    inputs.wristConnected = wristDebounce.calculate(wristStatus.isOK());

    inputs.wristMotorAngle = Degrees.of(wristPosition.getValue().magnitude() * 360 / GEAR_RATIO);

    inputs.wristVelocity =
        DegreesPerSecond.of(wristVelocity.getValue().magnitude() * 360 / GEAR_RATIO);
    inputs.intakeVelocity =
        DegreesPerSecond.of(intakeVelocity.getValue().magnitude() * 360 / GEAR_RATIO);

    inputs.wristAppliedVoltage = wristAppliedVolts.getValue();
    inputs.wristStatorCurrent = wristStatorCurrent.getValue();
    inputs.wristSupplyCurrent = wristSupplyCurrent.getValue();
    inputs.intakeAppliedVoltage = intakeAppliedVolts.getValue();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValue();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValue();

    inputs.setpoint = SetAngle;
    inputs.wristManual = wristManual;
    inputs.intakePercent = runPercent;

    inputs.angle = Degrees.of((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO));

    tempPIDTuning();

    if (locked) {
      wrist.setVoltage(wristPID.calculate(inputs.angle.magnitude()));
    }

    // If intake drawing too much current, algae is in
    if (inputs.intakeStatorCurrent.magnitude() > 40) {
      inputs.isAlgaeIn = true;
    } else {
      inputs.isAlgaeIn = false;
    }
  }

  @Override
  public void setAngle(Angle angle) {
    SetAngle = angle;
    wristPID.setGoal(angle.magnitude());
    locked = true;
  }

  @Override
  public void stopHere() {
    wristPID.reset(((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO)), 0);
    SetAngle = Degrees.of((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO));
    locked = true;
  }

  @Override
  public void wristManual(double power) {
    locked = false;
    wristManual = power;
    wrist.set(wristManual);
  }

  @Override
  public void runClaw(double power) {
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
