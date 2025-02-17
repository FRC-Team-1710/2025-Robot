// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of the arm subsystem. This class extends ArmIOCTRE to provide a
 * physics-based simulation of the arm mechanism using WPILib's simulation classes.
 *
 * <p>The simulation models: - Dual Kraken X60 FOC motors - Realistic arm physics including gravity
 * and moment of inertia - Position and velocity feedback through simulated encoders - Battery
 * voltage effects - Motion limits (0° to 180°)
 */
public class ArmIOSIM extends ArmIOCTRE {

  /** Physics simulation model for the arm mechanism */
  private final SingleJointedArmSim motorSimModel;

  /** Simulation state for the leader motor */
  private final TalonFXSimState leaderSim;
  /** Simulation state for the follower motor */
  private final TalonFXSimState followerSim;
  /** Simulation state for the CANcoder */
  private final CANcoderSimState encoderSim;

  /** Constructs a new ArmIOSIM instance. */
  public ArmIOSIM() {
    super(); // Initialize hardware interface components

    // Get simulation states for all hardware
    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();
    encoderSim = encoder.getSimState();

    // Configure dual Kraken X60 FOC motors
    DCMotor motor = DCMotor.getKrakenX60Foc(2);

    // Define arm physical properties
    Distance armLength = Inches.of(12);
    Mass armMass = Pounds.of(15);

    // Calculate moment of inertia using WPILib helper
    double armMOI = SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms));

    // Create arm physics model
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createSingleJointedArmSystem(motor, armMOI, GEAR_RATIO);

    // Initialize arm simulation
    motorSimModel =
        new SingleJointedArmSim(
            linearSystem,
            motor,
            GEAR_RATIO,
            armLength.in(Meters),
            Degrees.of(0).in(Radians), // Lower limit (0°)
            Degrees.of(180).in(Radians), // Upper limit (180)
            true, // Enable gravity simulation
            Degrees.of(90).in(Radians)); // Start at 90°
  }

  /**
   * Updates the simulation model and all simulated sensor inputs.
   *
   * @param inputs The ArmIOInputs object to update with simulated values
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update base class inputs first
    super.updateInputs(inputs);

    // Simulate battery voltage effects on all devices
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update physics simulation
    motorSimModel.setInputVoltage(leaderSim.getMotorVoltage());
    motorSimModel.update(0.020); // Simulate 20ms timestep (50Hz)

    // Get position and velocity from physics simulation
    Angle position = Radians.of(motorSimModel.getAngleRads());
    AngularVelocity velocity = RadiansPerSecond.of(motorSimModel.getVelocityRadPerSec());

    // Update simulated motor encoder readings (accounts for gear ratio)
    leaderSim.setRawRotorPosition(position.times(GEAR_RATIO));
    leaderSim.setRotorVelocity(velocity.times(GEAR_RATIO));

    // Update simulated CANcoder readings (direct angle measurement)
    encoderSim.setRawPosition(position);
    encoderSim.setVelocity(velocity);
  }
}
