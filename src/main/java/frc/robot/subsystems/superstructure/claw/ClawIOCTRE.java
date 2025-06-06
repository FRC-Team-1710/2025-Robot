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

package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class ClawIOCTRE implements ClawIO {
  public static final double GEAR_RATIO = 66.6666666;
  private boolean locked = false;
  private boolean rollerLocked = false;
  private boolean hasZeroed = false;

  private double kP = 0.25;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kacel = 750;
  private double kvel = 250;

  private double RollerkP = 3;
  private double RollerkI = 0.0;
  private double RollerkD = 0.0;

  public final TalonFX wrist = new TalonFX(51);
  public final TalonFX rollers = new TalonFX(52);

  private final PIDController rollerPID = new PIDController(RollerkP, RollerkI, RollerkD);
  private final ProfiledPIDController wristPID =
      new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kvel, kacel));
  private final ArmFeedforward wristFF = new ArmFeedforward(kS, kG, kV, kA);

  private final StatusSignal<Angle> wristPosition = wrist.getPosition();
  private final StatusSignal<AngularVelocity> wristVelocity = wrist.getVelocity();
  private final StatusSignal<Voltage> wristAppliedVolts = wrist.getMotorVoltage();
  private final StatusSignal<Current> wristStatorCurrent = wrist.getStatorCurrent();
  private final StatusSignal<Current> wristSupplyCurrent = wrist.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> intakeVelocity = rollers.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = rollers.getMotorVoltage();
  private final StatusSignal<Current> intakeStatorCurrent = rollers.getStatorCurrent();
  private final StatusSignal<Current> intakeSupplyCurrent = rollers.getSupplyCurrent();

  private final Debouncer clawDebounce = new Debouncer(0.5);
  private final Debouncer wristDebounce = new Debouncer(0.5);

  private Angle SetAngle = Degrees.of(0);
  private double wristManual = 0.0;
  private double runPercent = 0.0;

  TalonFXConfiguration config = new TalonFXConfiguration();

  public ClawIOCTRE() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wrist.getConfigurator().apply(config);
    rollers.getConfigurator().apply(config);

    wrist.setPosition(0);

    SmartDashboard.putNumber("Claw/PID/P", kP);
    SmartDashboard.putNumber("Claw/PID/I", kI);
    SmartDashboard.putNumber("Claw/PID/D", kD);

    SmartDashboard.putNumber("Claw/RollerPID/P", RollerkP);
    SmartDashboard.putNumber("Claw/RollerPID/I", RollerkI);
    SmartDashboard.putNumber("Claw/RollerPID/D", RollerkD);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristPosition,
        wristVelocity,
        wristStatorCurrent,
        wristSupplyCurrent,
        intakeVelocity,
        intakeStatorCurrent,
        intakeSupplyCurrent);

    wristPID.setGoal(0);
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

    inputs.locked = locked;

    inputs.wristAppliedVoltage = wristAppliedVolts.getValue();
    inputs.wristStatorCurrent = wristStatorCurrent.getValue();
    inputs.wristSupplyCurrent = wristSupplyCurrent.getValue();
    inputs.rollerAppliedVoltage = intakeAppliedVolts.getValue();
    inputs.rollerStatorCurrent = intakeStatorCurrent.getValue();
    inputs.rollerSupplyCurrent = intakeSupplyCurrent.getValue();
    inputs.rollerPosition = rollers.getPosition().getValueAsDouble();

    inputs.setpoint = SetAngle;
    inputs.wristManual = wristManual;
    inputs.intakePercent = runPercent;

    inputs.rollerLocked = rollerLocked;
    inputs.angle = Degrees.of((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO));

    inputs.hasZeroed = hasZeroed;

    tempPIDTuning();

    SmartDashboard.putNumber("Claw/PIDSetpoint", wristPID.getSetpoint().position);
    SmartDashboard.putNumber("Claw/PIDGoal", wristPID.getGoal().position);
    SmartDashboard.putNumber("Claw/PIDPosition", inputs.angle.magnitude());

    if (locked) {
      if (inputs.killSwich) {
        wrist.stopMotor();
      } else {
        wrist.setVoltage(
            wristPID.calculate(inputs.angle.magnitude())
                + wristFF.calculate(inputs.angle.in(Radians), wristPID.getSetpoint().velocity));
      }
    }

    if (rollerLocked) {
      rollers.setVoltage(rollerPID.calculate(inputs.rollerPosition));
    }

    Logger.recordOutput("roller locked", rollerLocked);
  }

  @Override
  public void setAngle(Angle angle) {
    SetAngle = angle;
    wristPID.setGoal(angle.magnitude());
    locked = true;
  }

  @Override
  public void zero() {
    wrist.setPosition(0);
    wristPID.reset(0);
    hasZeroed = true;
  }

  @Override
  public void setBrake(boolean lock) {
    config.MotorOutput.NeutralMode = lock ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    rollers.getConfigurator().apply(config);
  }

  @Override
  public void lockRoller() {
    rollerPID.setSetpoint(rollers.getPosition().getValueAsDouble());
    rollerLocked = true;
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
    wrist.setVoltage(wristManual);
  }

  @Override
  public void setRollers(double power) {
    runPercent = power;
    rollerLocked = false;
    rollers.set(power);
  }

  @Override
  public void zeroPIDToAngle() {
    wristPID.reset((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO), 0);
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

    if (RollerkP != SmartDashboard.getNumber("Claw/RollerPID/P", RollerkP)) {
      RollerkP = SmartDashboard.getNumber("Claw/RollerPID/P", RollerkP);
      rollerPID.setP(RollerkP);
    }

    if (RollerkI != SmartDashboard.getNumber("Claw/RollerPID/I", RollerkI)) {
      RollerkI = SmartDashboard.getNumber("Claw/RollerPID/I", RollerkI);
      rollerPID.setI(RollerkI);
    }

    if (RollerkD != SmartDashboard.getNumber("Claw/RollerPID/D", RollerkD)) {
      RollerkD = SmartDashboard.getNumber("Claw/RollerPID/D", RollerkD);
      rollerPID.setD(RollerkD);
    }
  }
}
