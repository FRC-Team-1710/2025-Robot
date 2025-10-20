package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.Conversions;

/**
 * CTRE-based implementation of the ElevatorIO interface for controlling an elevator mechanism. This
 * implementation uses TalonFX motors and a CANcoder for position feedback. The elevator consists of
 * a leader motor, a follower motor, and an encoder for precise positioning.
 */
public class ElevatorIOCTRE implements ElevatorIO {
  /** The gear ratio between the motor and the elevator mechanism */
  public static final double GEAR_RATIO = 6.0;

  private boolean locked = false;

  /** The leader TalonFX motor controller (CAN ID: 11) */
  public final TalonFX leader = new TalonFX(11);

  /** The follower TalonFX motor controller (CAN ID: 12) */
  public final TalonFX follower = new TalonFX(12);

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

  // FAST
  // private double kP = 0.1; // 0.1 // 0.75
  // private double kI = 0; // 0.0 // 0.0
  // private double kD = 0; // 0.0 // 0.0
  // private double kS = 0.1875; // 0.0 // 0.1
  // private double kG = 0.315; // 0.3375 // 0.375
  // private double kV = 0.126; // 0.0 // 0.075
  // private double kA = 0; // 0.0 // 0.0
  // private double kAcel = 125; // 200
  // private double kVel = 175;

  // slow
  private double kP = 0.05; // 0.1 // 0.75
  private double kI = 0; // 0.0 // 0.0
  private double kD = 0; // 0.0 // 0.0
  private double kS = 0.1875; // 0.0 // 0.1
  private double kG = 0.31; // 0.3375 // 0.375
  private double kV = 0.1275; // 0.0 // 0.075
  private double kA = 0; // 0.0 // 0.0
  private double kAcel = 65; // 200
  private double kVel = 175;

  // Status signals for monitoring motor and encoder states
  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<Angle> leaderRotorPosition = leader.getRotorPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<AngularVelocity> leaderRotorVelocity = leader.getRotorVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Angle> followerPosition = follower.getPosition();
  private final StatusSignal<Angle> followerRotorPosition = follower.getRotorPosition();
  private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
  private final StatusSignal<AngularVelocity> followerRotorVelocity = follower.getRotorVelocity();
  private final StatusSignal<Voltage> followerAppliedVolts = follower.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Double> leaderSetpoint = leader.getClosedLoopReference();

  // Debouncers for connection status (filters out brief disconnections)
  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer followerDebounce = new Debouncer(0.5);

  private Distance setpoint = Inches.of(0);

  /**
   * The radius of the elevator pulley/drum, used for converting between rotations and linear
   * distance
   */
  protected final Distance elevatorRadius = Inches.of(1.1338619402985);

  protected final Distance cancoderTripThreshold = Inches.of(15);

  /**
   * Constructs a new ElevatorIOCTRE instance and initializes all hardware components. This includes
   * configuring both motors, setting up the follower relationship, and optimizing CAN bus
   * utilization for all devices.
   */
  public ElevatorIOCTRE() {
    if (Constants.useSmartDashboard) {
      SmartDashboard.putNumber("Elevator/PID/P", kP);
      SmartDashboard.putNumber("Elevator/PID/I", kI);
      SmartDashboard.putNumber("Elevator/PID/D", kD);
      SmartDashboard.putNumber("Elevator/PID/S", kS);
      SmartDashboard.putNumber("Elevator/PID/G", kG);
      SmartDashboard.putNumber("Elevator/PID/V", kV);
      SmartDashboard.putNumber("Elevator/PID/A", kA);
      SmartDashboard.putNumber("Elevator/PID/Acel", kAcel);
      SmartDashboard.putNumber("Elevator/PID/Vel", kVel);
      SmartDashboard.putBoolean("Zero", false);
      SmartDashboard.putBoolean("ELEUPD", false);
    }

    // Set up follower to mirror leader
    follower.setControl(new Follower(leader.getDeviceID(), true));

    // Configure both motors with identical settings
    TalonFXConfiguration config = createMotorConfiguration();
    leader.getConfigurator().apply(config);

    // Configure update frequencies for all status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50Hz update rate
        leaderPosition,
        leaderRotorPosition,
        leaderVelocity,
        leaderRotorVelocity,
        leaderAppliedVolts,
        followerPosition,
        followerRotorPosition,
        followerVelocity,
        followerRotorVelocity,
        followerAppliedVolts,
        leaderStatorCurrent,
        followerStatorCurrent,
        leaderSupplyCurrent,
        followerSupplyCurrent,
        leaderSetpoint);

    // Optimize CAN bus usage for all devices
    leader.optimizeBusUtilization(4, 0.1);
    follower.optimizeBusUtilization(4, 0.1);

    follower.setPosition(0);
    leader.setPosition(0);
  }

  /**
   * Creates the motor configuration with appropriate settings. Sets up neutral mode, PID gains, and
   * feedback device configuration.
   *
   * @return The configured TalonFXConfiguration object
   */
  private TalonFXConfiguration createMotorConfiguration() {
    var config = new TalonFXConfiguration();
    if (Constants.useSmartDashboard) {
      config.Slot0.kP = SmartDashboard.getNumber("Elevator/PID/P", kP);
      config.Slot0.kI = SmartDashboard.getNumber("Elevator/PID/I", kI);
      config.Slot0.kD = SmartDashboard.getNumber("Elevator/PID/D", kD);
      config.Slot0.kS = SmartDashboard.getNumber("Elevator/PID/S", kS);
      config.Slot0.kG = SmartDashboard.getNumber("Elevator/PID/G", kG);
      config.Slot0.kV = SmartDashboard.getNumber("Elevator/PID/V", kV);
      config.Slot0.kA = SmartDashboard.getNumber("Elevator/PID/A", kA);
      config.MotionMagic.MotionMagicAcceleration =
          SmartDashboard.getNumber("Elevator/PID/Acel", kAcel);
      config.MotionMagic.MotionMagicCruiseVelocity =
          SmartDashboard.getNumber("Elevator/PID/Vel", kVel);
    } else {
      config.Slot0.kP = kP;
      config.Slot0.kI = kI;
      config.Slot0.kD = kD;
      config.Slot0.kS = kS;
      config.Slot0.kG = kG;
      config.Slot0.kV = kV;
      config.Slot0.kA = kA;
      config.MotionMagic.MotionMagicAcceleration = kAcel;
      config.MotionMagic.MotionMagicCruiseVelocity = kVel;
    }
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    return config;
  }

  /**
   * Updates the elevator's input values with the latest sensor readings. This includes position,
   * velocity, voltage, and current measurements from both motors and the encoder, as well as
   * connection status for all devices.
   *
   * @param inputs The ElevatorIOInputs object to update with the latest values
   */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refresh all sensor data
    StatusCode leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderRotorPosition,
            leaderVelocity,
            leaderRotorVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent,
            leaderSetpoint);

    StatusCode followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition,
            followerRotorPosition,
            followerVelocity,
            followerRotorVelocity,
            followerAppliedVolts,
            followerStatorCurrent,
            followerSupplyCurrent);

    // Update connection status with debouncing
    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerDebounce.calculate(followerStatus.isOK());

    // Update position and velocity measurements
    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderRotorPosition = leaderRotorPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderRotorVelocity = leaderRotorVelocity.getValue();

    // Update voltage and current measurements
    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();

    inputs.goal = setpoint;
    inputs.setpoint = leaderSetpoint.getValue();

    inputs.locked = locked;

    if (Constants.useSmartDashboard) {
      SmartDashboard.putNumber("Elevator Inches", leader.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(
          "Elevator Setpoint", leader.getClosedLoopReference().getValueAsDouble());
      tempPIDTuning();
    }
  }

  /**
   * Sets the desired distance for the elevator to move to. Converts the desired linear distance to
   * encoder rotations and applies position control.
   *
   * @param distance The target distance for the elevator
   */
  @Override
  public void setDistance(Distance distance) {
    leader.setControl(
        request.withPosition(Conversions.metersToRotations(distance, GEAR_RATIO, elevatorRadius)));
    locked = true;
  }

  @Override
  public void stopHere() {
    leader.setControl(request.withPosition(leader.getPosition().getValue()));
    locked = true;
  }

  @Override
  public void setManual(double power) {
    leader.setControl(new VoltageOut(power));
    locked = false;
  }

  @Override
  public void zero() {
    // TODO
  }

  /**
   * Stops all elevator movement by stopping the leader motor. The follower will also stop due to
   * the follower relationship.
   */
  @Override
  public void stop() {
    leader.stopMotor();
    locked = false;
  }

  private void tempPIDTuning() {
    if (SmartDashboard.getBoolean("ELEUPD", false)) {
      SmartDashboard.putBoolean("ELEUPD", false);
      leader.getConfigurator().apply(createMotorConfiguration());
      follower.getConfigurator().apply(createMotorConfiguration());
    }
  }
}
