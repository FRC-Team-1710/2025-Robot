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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
  private boolean stop = false;

  private double kP = 0.5;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0.0;
  private double kG = 0.3564;
  private double kV = 3.18;
  private double kA = 0.0;
  private double kVel = 3;
  private double kAcel = 4.5;

  public final TalonFX leader = new TalonFX(30);
  public final TalonFX follower = new TalonFX(31);
  public final LaserCan laserCan = new LaserCan(32);

  private final TrapezoidProfile.Constraints m_Constraints =
      new TrapezoidProfile.Constraints(0.3, 0.3); // MAX velocity, MAX aceleration
  private final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(0, 0, 0, m_Constraints);
  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 0, 0, 0);

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();

  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer followerDebounce = new Debouncer(0.5);

  protected final Distance elevatorRadius = Inches.of(1.5);

  private Distance SetPoint = Meters.of(0);
  private double spinManual = 0.0;

  public ElevatorIOCTRE() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leader.getDeviceID(), true));

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
        followerStatorCurrent,
        leaderSupplyCurrent,
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

    var followerStatus = BaseStatusSignal.refreshAll(followerStatorCurrent, followerSupplyCurrent);

    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerDebounce.calculate(followerStatus.isOK());

    inputs.leaderPosition = leaderPosition.getValue().div(GEAR_RATIO);
    inputs.leaderVelocity = leaderVelocity.getValue().div(GEAR_RATIO);

    inputs.setpoint = SetPoint;
    inputs.manualSpin = spinManual;

    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();

    if (laserCan.getMeasurement() == null) {
      inputs.laserFault = true;
      inputs.distance =
          Meters.of(
              (leaderPosition.getValueAsDouble() / GEAR_RATIO)
                  * (Units.inchesToMeters(1.5) * Math.PI));
    } else {
      inputs.laserFault = false;
      inputs.laserDistance =
          Meters.of(Double.valueOf(laserCan.getMeasurement().distance_mm) / 1000);
      inputs.distance = Meters.of(Double.valueOf(laserCan.getMeasurement().distance_mm) / 1000);
    }

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
      } else {
        leader.setVoltage(elevatorFF.getKg() + spinManual);
      }
    }
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
