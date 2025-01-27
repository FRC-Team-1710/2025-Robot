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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
  private boolean stop = false;

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
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("S", kS);
    SmartDashboard.putNumber("G", kG);
    SmartDashboard.putNumber("V", kV);
    SmartDashboard.putNumber("A", kA);
    SmartDashboard.putNumber("Acel", kAcel);
    SmartDashboard.putNumber("Vel", kVel);

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
    SmartDashboard.putNumber("Elevator Leader Position", leaderPosition.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Leader Velocity", leaderVelocity.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Follower Position", followerPosition.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Follower Velocity", followerVelocity.getValue().magnitude());

    inputs.setpoint = SetPoint;
    inputs.manualSpin = spinManual;
    
    SmartDashboard.putNumber("Elevator Leader Applied Volt", leaderAppliedVolts.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Leader Stator Current", leaderStatorCurrent.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Leader Supply Current", leaderSupplyCurrent.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Follower Applied Volt", followerAppliedVolts.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Follower Stator Current", followerStatorCurrent.getValue().magnitude());
    SmartDashboard.putNumber("Elevator Follower Supply Current", followerSupplyCurrent.getValue().magnitude());
    inputs.leaderAppliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.followerAppliedVoltage = followerAppliedVolts.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();

    inputs.distance = Meters.of((leaderPosition.getValueAsDouble() / GEAR_RATIO) * (Units.inchesToMeters(2.383) * Math.PI));

    leader.optimizeBusUtilization(4, 0.1);
    follower.optimizeBusUtilization(4, 0.1);
  }

  @Override
  public void updatePID(ElevatorIOInputsAutoLogged inputs) {
    tempPIDTuning();
    if (!stop) {
      if (locked) {
        leader.setVoltage(
            elevatorPID.calculate(inputs.distance.magnitude())
                + elevatorFF.calculate(elevatorPID.getSetpoint().velocity));
        follower.setVoltage(
            elevatorPID.calculate(inputs.distance.magnitude())
                + elevatorFF.calculate(elevatorPID.getSetpoint().velocity));        
      } 
    }
    SmartDashboard.putNumber("Elevator/Distance INCHES", Units.metersToInches(inputs.distance.magnitude()));
    SmartDashboard.putNumber("Elevator/Pos", inputs.distance.magnitude());
    SmartDashboard.putNumber("Elevator/Goal", elevatorPID.getGoal().position);
    SmartDashboard.putNumber("Elevator/Setpoint", elevatorPID.getSetpoint().position);
  }

  @Override
  public void setDistance(Distance distance) {
    SetPoint = distance;
    locked = true;
    stop = false;
    ElevatorIOInputs.SIMsetpoint = null;
  }

  @Override
  public void setManual(double power) {
    locked = false;
    spinManual = power;
    leader.set(spinManual);
    follower.set(-spinManual);
  }

  @Override
  public void resetPID() {
    locked = false;
    elevatorPID.reset((leaderPosition.getValueAsDouble()) * (Units.inchesToMeters(1.5) * Math.PI));
  }

  @Override
  public void resetEncoder() {
    leader.setPosition(0);
    follower.setPosition(0);
  }

  @Override
  public void stop() {
    stop = true;
    leader.stopMotor();
  }

  private void tempPIDTuning() {
    if (kP != SmartDashboard.getNumber("P", kP)) {
      kP = SmartDashboard.getNumber("P", kP);
      elevatorPID.setP(kP);
    }

    if (kI != SmartDashboard.getNumber("I", kI)) {
      kI = SmartDashboard.getNumber("I", kI);
      elevatorPID.setI(kI);
    }

    if (kD != SmartDashboard.getNumber("D", kD)) {
      kD = SmartDashboard.getNumber("D", kD);
      elevatorPID.setD(kD);
    }

    if (kS != SmartDashboard.getNumber("S", kS)) {
      kS = SmartDashboard.getNumber("S", kS);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kG != SmartDashboard.getNumber("G", kG)) {
      kG = SmartDashboard.getNumber("G", kG);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kV != SmartDashboard.getNumber("V", kV)) {
      kV = SmartDashboard.getNumber("V", kV);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kA != SmartDashboard.getNumber("A", kA)) {
      kA = SmartDashboard.getNumber("A", kA);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kAcel != SmartDashboard.getNumber("Acel", kAcel)) {
      kAcel = SmartDashboard.getNumber("Acel", kAcel);
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }

    if (kVel != SmartDashboard.getNumber("Vel", kVel)) {
      kVel = SmartDashboard.getNumber("Vel", kVel);
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }
  }
}
