// This is my pride and joy :)
//   -Carter

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.superstructure.elevator.ElevatorEncoder.EncoderType;
import frc.robot.utils.Conversions;

/**
 * CTRE-based implementation of the ElevatorIO interface for controlling an elevator mechanism. This
 * implementation uses TalonFX motors and a CANcoder for position feedback. The elevator consists of
 * a leader motor, a follower motor, and an encoder for precise positioning.
 */
public class ElevatorIOCTRE implements ElevatorIO {

  private final EncoderType encoderType = EncoderType.motorEncoders;

  /** The gear ratio between the motor and the elevator mechanism */
  public static final double GEAR_RATIO = 6.0;

  /** Whoops */
  public boolean encoderFault = false;

  /** The leader TalonFX motor controller (CAN ID: 11) */
  public final TalonFX leader = new TalonFX(11);

  /** The follower TalonFX motor controller (CAN ID: 12) */
  public final TalonFX follower = new TalonFX(12);

  /** The encoder that can be swiched with one variable */
  private final ElevatorEncoder encoder = new ElevatorEncoder(encoderType, leader);

  private double kP = 1.125; // 0.1 // 0.75
  private double kI = 0; // 0.0 // 0.0
  private double kD = 0.005; // 0.0 // 0.0
  private double kS = 0.0; // 0.0 // 0.1
  private double kG = 0.3375; // 0.3375 // 0.375
  private double kV = 0; // 0.0 // 0.075
  private double kA = 0; // 0.0 // 0.0
  private double kAcel = 250; // 200
  private double kVel = 250;

  private boolean locked = false;

  private boolean zeroed = true;

  private final TrapezoidProfile.Constraints m_Constraints;
  private final ProfiledPIDController elevatorPID;
  private ElevatorFeedforward elevatorFF;

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
        followerSupplyCurrent);

    // Optimize CAN bus usage for all devices
    leader.optimizeBusUtilization(4, 0.1);
    follower.optimizeBusUtilization(4, 0.1);

    follower.setPosition(0);
    leader.setPosition(0);

    m_Constraints = new TrapezoidProfile.Constraints(kVel, kAcel); // MAX velocity, MAX aceleration
    elevatorPID = new ProfiledPIDController(kP, kI, kD, m_Constraints);
    elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);

    if (encoder.isMotorEncoders()) {
      encoderFault = true;
    }

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
  }

  /**
   * Creates the motor configuration with appropriate settings. Sets up neutral mode, PID gains, and
   * feedback device configuration.
   *
   * @return The configured TalonFXConfiguration object
   */
  private TalonFXConfiguration createMotorConfiguration() {
    var config = new TalonFXConfiguration();
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
            leaderSupplyCurrent);

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

    inputs.encoderFault = encoderFault;

    inputs.elevatorDistance = decideEncoderStatus();

    inputs.elevatorSetpoint = setpoint;

    SmartDashboard.putNumber("Elevator Inches", inputs.elevatorDistance.in(Inches));
    SmartDashboard.putNumber("Elevator Setpoint", elevatorPID.getSetpoint().position);
    SmartDashboard.putNumber("Elevator Goal", elevatorPID.getGoal().position);
    tempPIDTuning();

    if (locked) {
      if (inputs.killSwich || !zeroed) {
        leader.stopMotor();
      } else {
        leader.setVoltage(
            (elevatorPID.calculate(inputs.elevatorDistance.in(Inches))
                + elevatorFF.calculate(elevatorPID.getSetpoint().velocity)));
      }
      inputs.manual = 0.0;
    } else {
      inputs.manual = leader.getMotorVoltage().getValueAsDouble();
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
    if (!encoderFault && !encoder.isMotorEncoders()) {
      leader.setPosition(
          Conversions.metersToRotations(encoder.getDistance(), GEAR_RATIO, elevatorRadius));
    }
    elevatorPID.setGoal(distance.in(Inches));
    setpoint = distance;
    locked = true;
  }

  private Distance decideEncoderStatus() {
    if (!encoderFault && encoderType == EncoderType.stringEncoder) {
      if ((encoder.getDistance().isNear(motorDistance(), cancoderTripThreshold))) {
        return encoder.getDistance();
      } else {
        encoderFault = true;
        zeroed = false;
      }
    }
    return encoder.isCancoder() ? motorDistance() : encoder.getDistance();
  }

  private Distance motorDistance() {
    return Conversions.rotationsToDistance(leaderPosition.getValue(), GEAR_RATIO, elevatorRadius);
  }

  @Override
  public void stopHere() {
    elevatorPID.reset(encoder.getDistance().in(Inches), 0);
    setpoint = encoder.getDistance();
    locked = true;
  }

  @Override
  public void setManual(double power) {
    leader.setVoltage(power * 12);
    if (power == 0) {
      elevatorPID.reset(encoder.getDistance().in(Inches));
    }
    locked = false;
  }

  @Override
  public void zero() {
    encoder.zero();
    leader.setPosition(0);
    elevatorPID.reset(0);
    zeroed = true;
  }

  /**
   * Stops all elevator movement by stopping the leader motor. The follower will also stop due to
   * the follower relationship.
   */
  @Override
  public void stop() {
    locked = false;
    leader.stopMotor();
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
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kVel, kAcel));
    }

    if (kVel != SmartDashboard.getNumber("Elevator/PID/Vel", kVel)) {
      kVel = SmartDashboard.getNumber("Elevator/PID/Vel", kVel);
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kVel, kAcel));
    }
  }
}
