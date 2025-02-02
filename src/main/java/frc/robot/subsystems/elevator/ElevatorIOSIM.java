package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Conversions;

/**
 * Simulation implementation of the elevator subsystem. This class extends ElevatorIOCTRE to provide
 * a physics-based simulation of the elevator mechanism using WPILib's simulation classes.
 *
 * <p>The simulation models: - Dual Kraken X60 FOC motors - Realistic elevator physics including
 * gravity - Position and velocity feedback through simulated encoders - Battery voltage effects
 */
public class ElevatorIOSIM extends ElevatorIOCTRE {

  /** Physics simulation model for the elevator mechanism */
  private final ElevatorSim motorSimModel;

  /** Simulation state for the leader motor */
  private final TalonFXSimState leaderSim;
  /** Simulation state for the follower motor */
  private final TalonFXSimState followerSim;

  /**
   * Constructs a new ElevatorIOSIM instance. Initializes the physics simulation with realistic
   * parameters including: - Dual Kraken X60 FOC motors - 10 pound carriage mass - 8 foot maximum
   * height - Gravity simulation enabled
   */
  public ElevatorIOSIM() {
    super(); // Initialize hardware interface components

    // Get simulation states for all hardware
    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();

    // Configure dual Kraken X60 FOC motors
    DCMotor motor = DCMotor.getKrakenX60(2);

    // Create elevator physics model
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createElevatorSystem(
            motor,
            Pounds.of(25).in(Kilograms), // Carriage mass (10 lbs -> kg)
            elevatorRadius.in(Meters), // Drum radius in meters
            GEAR_RATIO); // Motor to mechanism gear ratio

    // Initialize elevator simulation
    motorSimModel =
        new ElevatorSim(
            linearSystem,
            motor,
            0, // Initial position
            Feet.of(7).in(Meters), // Maximum height (7 feet -> meters)
            true, // Enable gravity simulation
            0); // Start at bottom position
  }

  /**
   * Updates the simulation model and all simulated sensor inputs.
   *
   * @param inputs The ElevatorIOInputs object to update with simulated values
   */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update base class inputs first
    super.updateInputs(inputs);

    // Simulate battery voltage effects on all devices
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update physics simulation
    motorSimModel.setInputVoltage(leaderSim.getMotorVoltage());
    motorSimModel.update(0.020); // Simulate 20ms timestep (50Hz)

    // Convert linear position/velocity to rotational units for based on encoder
    Angle position =
        Conversions.metersToRotations(
            Meters.of(motorSimModel.getPositionMeters()), 6, elevatorRadius);

    SmartDashboard.putNumber("Elevator Position", inputs.elevatorDistance.magnitude());

    // Convert linear velocity to angular velocity based on encoder
    AngularVelocity velocity =
        Conversions.metersToRotationsVel(
            MetersPerSecond.of(motorSimModel.getVelocityMetersPerSecond()), 6, elevatorRadius);

    // Update simulated motor readings converts through gear ratio
    leaderSim.setRawRotorPosition(position.times(GEAR_RATIO));
    leaderSim.setRotorVelocity(velocity.times(GEAR_RATIO));
  }
}
