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

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOCTRE implements ElevatorIO {
  public static final double GEAR_RATIO = 6.0;
  private boolean locked = false;
  private double drumSize = Units.inchesToMeters(2.383);

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kVel = 0.3;
  private double kAcel = 0.3;

  public final TalonFX leader = new TalonFX(11);
  public final TalonFX follower = new TalonFX(12);

  private final TrapezoidProfile.Constraints m_Constraints =
      new TrapezoidProfile.Constraints(kVel, kAcel); // MAX velocity, MAX aceleration
  private final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(kP, kI, kD, m_Constraints);
  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Angle> followerPosition = follower.getPosition();
  private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
  private final StatusSignal<Voltage> followerAppliedVolts = follower.getMotorVoltage();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();

  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer followerDebounce = new Debouncer(0.5);

  private Distance SetPoint = Meters.of(0);
  private double spinManual = 0.0;

  public ElevatorIOCTRE() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leader.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    follower.getConfigurator().apply(config);

    SmartDashboard.putNumber("Elevator/PID/P", kP);
    SmartDashboard.putNumber("Elevator/PID/I", kI);
    SmartDashboard.putNumber("Elevator/PID/D", kD);
    SmartDashboard.putNumber("Elevator/PID/S", kS);
    SmartDashboard.putNumber("Elevator/PID/G", kG);
    SmartDashboard.putNumber("Elevator/PID/V", kV);
    SmartDashboard.putNumber("Elevator/PID/A", kA);
    SmartDashboard.putNumber("Elevator/PID/Acel", kAcel);
    SmartDashboard.putNumber("Elevator/PID/Vel", kVel);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        followerPosition,
        followerVelocity,
        followerStatorCurrent,
        followerSupplyCurrent);
    
    leader.optimizeBusUtilization(4, 0.1);
    follower.optimizeBusUtilization(4, 0.1);

    resetEncoder(); // Sets current position to 1 inch
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);

    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition,
            followerVelocity,
            followerAppliedVolts,
            followerStatorCurrent,
            followerSupplyCurrent);

    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerDebounce.calculate(followerStatus.isOK());

    inputs.leaderPosition = leaderPosition.getValue().div(GEAR_RATIO);
    inputs.leaderVelocity = leaderVelocity.getValue().div(GEAR_RATIO);
    inputs.followerPosition = followerPosition.getValue().div(GEAR_RATIO);
    inputs.followerVelocity = followerVelocity.getValue().div(GEAR_RATIO);

    inputs.leaderAppliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.followerAppliedVoltage = followerAppliedVolts.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();

    inputs.setpoint = SetPoint;
    inputs.manualSpin = spinManual;

    inputs.distance =
        Meters.of((leaderPosition.getValueAsDouble() / GEAR_RATIO) * (drumSize * Math.PI));

    SmartDashboard.putNumber(
        "Elevator/Motors/Leader/Position", leaderPosition.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Leader/Velocity", leaderVelocity.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Follower/Position", followerPosition.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Follower/Velocity", followerVelocity.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Leader/Applied Volt", leaderAppliedVolts.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Leader/Stator Current", leaderStatorCurrent.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Leader/Supply Current", leaderSupplyCurrent.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Follower/Applied Volt", followerAppliedVolts.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Follower/Stator Current", followerStatorCurrent.getValue().magnitude());
    SmartDashboard.putNumber(
        "Elevator/Motors/Follower/Supply Current", followerSupplyCurrent.getValue().magnitude());
  }

  @Override
  public void updatePID(ElevatorIOInputsAutoLogged inputs) {
    tempPIDTuning();
    if (locked) {
      leader.setVoltage(
          elevatorPID.calculate(inputs.distance.magnitude())
              + elevatorFF.calculate(elevatorPID.getSetpoint().velocity));
      follower.setVoltage(
          elevatorPID.calculate(inputs.distance.magnitude())
              + elevatorFF.calculate(elevatorPID.getSetpoint().velocity));
    }
    SmartDashboard.putNumber(
        "Elevator/Distance INCHES", Units.metersToInches(inputs.distance.magnitude()));
    SmartDashboard.putNumber("Elevator/Distance METERS", inputs.distance.magnitude());
    SmartDashboard.putNumber("Elevator/PID/Goal", elevatorPID.getGoal().position);
    SmartDashboard.putNumber("Elevator/Setpoint", elevatorPID.getSetpoint().position);
  }

  @Override
  public void setDistance(Distance distance) {
    SetPoint = distance;
    locked = true;
    ElevatorIOInputs.SIMsetpoint = null;
  }

  @Override
  public void setManual(double power) {
    locked = false;
    spinManual = power;
    leader.set(spinManual);
    follower.set(spinManual);
  }

  @Override
  public void resetPID() {
    locked = false;
    elevatorPID.reset((leaderPosition.getValueAsDouble()) * (drumSize * Math.PI));
  }

  @Override
  public void resetEncoder() {
    leader.setPosition(6 / (2.383 * Math.PI)); // One inch above base to account for second stage
    follower.setPosition(6 / (2.383 * Math.PI));
  }

  private void tempPIDTuning() {
    if (kP != SmartDashboard.getNumber("Elevator/PID/P", kP)) {
      kP = SmartDashboard.getNumber("Elevator/PID/P", kP);
      elevatorPID.setP(kP);
    }

    if (kI != SmartDashboard.getNumber("Elevator/PID/I", kI)) {
      kI = SmartDashboard.getNumber("Elevator/PID/I", kI);
      elevatorPID.setI(kI);
    }

    if (kD != SmartDashboard.getNumber("Elevator/PID/D", kD)) {
      kD = SmartDashboard.getNumber("Elevator/PID/D", kD);
      elevatorPID.setD(kD);
    }

    if (kS != SmartDashboard.getNumber("Elevator/PID/S", kS)) {
      kS = SmartDashboard.getNumber("Elevator/PID/S", kS);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kG != SmartDashboard.getNumber("Elevator/PID/G", kG)) {
      kG = SmartDashboard.getNumber("Elevator/PID/G", kG);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kV != SmartDashboard.getNumber("Elevator/PID/V", kV)) {
      kV = SmartDashboard.getNumber("Elevator/PID/V", kV);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kA != SmartDashboard.getNumber("Elevator/PID/A", kA)) {
      kA = SmartDashboard.getNumber("Elevator/PID/A", kA);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kAcel != SmartDashboard.getNumber("Elevator/PID/Acel", kAcel)) {
      kAcel = SmartDashboard.getNumber("Elevator/PID/Acel", kAcel);
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }

    if (kVel != SmartDashboard.getNumber("Elevator/PID/Vel", kVel)) {
      kVel = SmartDashboard.getNumber("Elevator/PID/Vel", kVel);
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }
  }
}
