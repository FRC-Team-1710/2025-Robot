// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.funnel;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.Conversions;
import org.littletonrobotics.junction.Logger;

/**
 * CTRE-based implementation of the ArmIO interface for controlling a robot arm mechanism. This
 * implementation uses TalonFX motors and a CANcoder for position feedback. The arm consists of a
 * leader motor, a follower motor, and an encoder for precise angular positioning.
 */
public class FunnelIOCTRE implements FunnelIO {
  /** The gear ratio between the motor and the arm mechanism */
  public static final double GEAR_RATIO = 8;

  private boolean locked = false;

  /** The leader TalonFX motor controller (CAN ID: 20) */
  public final TalonFX leader = new TalonFX(31);

  /** The follower TalonFX motor controller (CAN ID: 21) */
  public final TalonFX angleMotor = new TalonFX(30);

  public final Servo aileron = new Servo(0);

  private final DigitalInput forwardBeamBreak = new DigitalInput(3);
  private final DigitalInput reverseBeamBreak = new DigitalInput(2);

  private double kP = 0.04;
  private double kI = 0.0;
  private double kD = 0.0025;
  private double kS = 0.0;
  private double kG = 1.125;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kVel = 150;
  private double kAcel = 200;

  // Status signals for monitoring motor and encoder states
  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Angle> angleMotorPosition = angleMotor.getPosition();
  private final StatusSignal<AngularVelocity> angleMotorVelocity = angleMotor.getVelocity();
  private final StatusSignal<Voltage> angleMotorAppliedVolts = angleMotor.getMotorVoltage();
  private final StatusSignal<Current> angleMotorStatorCurrent = angleMotor.getStatorCurrent();
  private final StatusSignal<Current> angleMotorSupplyCurrent = angleMotor.getSupplyCurrent();

  private TrapezoidProfile.Constraints constraints;
  private ProfiledPIDController anglePID;
  private ArmFeedforward angleFF;

  // Debouncers for connection status (filters out brief disconnections)
  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer angleMotorDebounce = new Debouncer(0.5);

  public Timer timer = new Timer();

  /**
   * Constructs a new ArmIOCTRE instance and initializes all hardware components. This includes
   * configuring both motors, setting up the follower relationship, and optimizing CAN bus
   * utilization for all devices.
   */
  public FunnelIOCTRE() {
    // Set up follower to mirror leader
    // follower.setControl(new Follower(leader.getDeviceID(), true));

    // Configure both motors with identical settings
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Audio.AllowMusicDurDisable = true;
    leader.getConfigurator().apply(config);

    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    angleConfig.Audio.AllowMusicDurDisable = true;
    angleMotor.getConfigurator().apply(angleConfig);

    constraints = new TrapezoidProfile.Constraints(kVel, kAcel);
    anglePID = new ProfiledPIDController(kP, kI, kD, constraints);
    angleFF = new ArmFeedforward(kS, kG, kV, kA);

    // Configure update frequencies for all status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50Hz update rate
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        angleMotorPosition,
        angleMotorVelocity,
        angleMotorAppliedVolts,
        angleMotorStatorCurrent,
        angleMotorSupplyCurrent);

    // Optimize CAN bus usage for all devices
    leader.optimizeBusUtilization(4, 0.1);
    angleMotor.optimizeBusUtilization(4, 0.1);

    if (Constants.useSmartDashboard) {
      SmartDashboard.putNumber("Funnel/PID/P", kP);
      SmartDashboard.putNumber("Funnel/PID/I", kI);
      SmartDashboard.putNumber("Funnel/PID/D", kD);
      SmartDashboard.putNumber("Funnel/PID/S", kS);
      SmartDashboard.putNumber("Funnel/PID/G", kG);
      SmartDashboard.putNumber("Funnel/PID/V", kV);
      SmartDashboard.putNumber("Funnel/PID/A", kA);
      SmartDashboard.putNumber("Funnel/PID/Acel", kAcel);
      SmartDashboard.putNumber("Funnel/PID/Vel", kVel);
    }

    angleMotor.setPosition(0);
    aileron.setAngle(FunnelConstants.AILERON_OUT);
  }

  /**
   * Updates the arm's input values with the latest sensor readings. This includes position,
   * velocity, voltage, and current measurements from both motors and the encoder, as well as
   * connection status for all devices.
   *
   * @param inputs The ArmIOInputs object to update with the latest values
   */
  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    if (Constants.useSmartDashboard) {
      tempPIDTuning();
    }

    // Refresh all sensor data
    StatusCode leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);

    StatusCode angleMotorStatus =
        BaseStatusSignal.refreshAll(
            angleMotorPosition,
            angleMotorVelocity,
            angleMotorAppliedVolts,
            angleMotorStatorCurrent,
            angleMotorSupplyCurrent);

    // Update connection status with debouncing
    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.angleMotorConnected = angleMotorDebounce.calculate(angleMotorStatus.isOK());

    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.angleMotorPosition = angleMotorPosition.getValue();
    inputs.angleMotorVelocity = angleMotorVelocity.getValue();
    inputs.angleMotorStatorCurrent = angleMotorStatorCurrent.getValue();
    inputs.angleMotorSupplyCurrent = angleMotorSupplyCurrent.getValue();

    // Calculate arm angle using encoder position
    inputs.funnelAngle = (inputs.angleMotorPosition.magnitude() * 360 / GEAR_RATIO);

    if (locked) {
      if (anglePID.getGoal().position != 0
          || (anglePID.getGoal().position == 0 && inputs.funnelAngle >= 10)) {
        angleMotor.setVoltage(
            (anglePID.calculate(inputs.funnelAngle)
                + angleFF.calculate(
                    Conversions.funnelAngleToFFRads(inputs.funnelAngle).magnitude(),
                    anglePID.getSetpoint().velocity)));
      } else {
        angleMotor.setVoltage(0);
        anglePID.reset(inputs.funnelAngle, 0);
      }
    }

    inputs.hasCoral = !forwardBeamBreak.get() || !reverseBeamBreak.get();

    if (Constants.useSmartDashboard) {
      SmartDashboard.putNumber("MOTOR", inputs.angleMotorPosition.magnitude());
      SmartDashboard.putNumber(
          "funnel/CORRECTEDPOSITION",
          Units.radiansToDegrees(Conversions.funnelAngleToFFRads(inputs.funnelAngle).magnitude()));
      SmartDashboard.putNumber("funnel/POSITION", inputs.funnelAngle);
      SmartDashboard.putNumber("funnel/GOAL", anglePID.getGoal().position);
      SmartDashboard.putNumber("funnel/SETPOINT", anglePID.getSetpoint().position);
      SmartDashboard.putNumber("Funnel/Aileron Angle", aileron.getAngle());
    }
    Logger.recordOutput("Funnel/Aileron Pos", aileron.getAngle());
  }

  /**
   * Sets the desired angle for the arm to move to. This should be based off encoder rotations to
   * arm.
   *
   * @param angle The target angle for the arm mechanism
   */
  @Override
  public void setPosition(Angle angle) {
    locked = true;
    anglePID.setGoal(angle.magnitude());
  }

  @Override
  public void zero() {
    locked = false;
    angleMotor.setPosition(0);
    anglePID.reset(0, 0);
  }

  @Override
  public void setRoller(double percent) {
    leader.set(percent);
  }

  /**
   * Stops all arm movement by stopping the leader motor. The follower will also stop due to the
   * follower relationship.
   */
  @Override
  public void stop() {
    leader.stopMotor();
    angleMotor.stopMotor();
  }

  public void setAileron(double angle) {
    aileron.setAngle(angle);
  }

  private void tempPIDTuning() {
    if (kP != SmartDashboard.getNumber("Funnel/PID/P", kP)) {
      kP = SmartDashboard.getNumber("Funnel/PID/P", kP);
      anglePID.setP(kP);
    }

    if (kI != SmartDashboard.getNumber("Funnel/PID/I", kI)) {
      kI = SmartDashboard.getNumber("Funnel/PID/I", kI);
      anglePID.setI(kI);
    }

    if (kD != SmartDashboard.getNumber("Funnel/PID/D", kD)) {
      kD = SmartDashboard.getNumber("Funnel/PID/D", kD);
      anglePID.setD(kD);
    }

    if (kS != SmartDashboard.getNumber("Funnel/PID/S", kS)) {
      kS = SmartDashboard.getNumber("Funnel/PID/S", kS);
      angleFF = new ArmFeedforward(kS, kG, kV, kA);
    }

    if (kG != SmartDashboard.getNumber("Funnel/PID/G", kG)) {
      kG = SmartDashboard.getNumber("Funnel/PID/G", kG);
      angleFF = new ArmFeedforward(kS, kG, kV, kA);
    }

    if (kV != SmartDashboard.getNumber("Funnel/PID/V", kV)) {
      kV = SmartDashboard.getNumber("Funnel/PID/V", kV);
      angleFF = new ArmFeedforward(kS, kG, kV, kA);
    }

    if (kA != SmartDashboard.getNumber("Funnel/PID/A", kA)) {
      kA = SmartDashboard.getNumber("Funnel/PID/A", kA);
      angleFF = new ArmFeedforward(kS, kG, kV, kA);
    }

    if (kAcel != SmartDashboard.getNumber("Funnel/PID/Acel", kAcel)) {
      kAcel = SmartDashboard.getNumber("Funnel/PID/Acel", kAcel);
      anglePID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }

    if (kVel != SmartDashboard.getNumber("Funnel/PID/Vel", kVel)) {
      kVel = SmartDashboard.getNumber("Funnel/PID/Vel", kVel);
      anglePID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }
  }
}
