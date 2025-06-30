package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

/**
 * Simulation implementation of the elevator subsystem. This class extends ElevatorIOCTRE to provide
 * a physics-based simulation of the elevator mechanism using WPILib's simulation classes.
 *
 * <p>The simulation models: - Dual Kraken X60 FOC motors - Realistic elevator physics including
 * gravity - Position and velocity feedback through simulated encoders - Battery voltage effects
 */
public class ElevatorIOSIM extends ElevatorIOCTRE {

  private double kP = 0.01;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0.0;
  private double kG = 0.2;
  private double kV = 0.051;
  private double kA = 0.0;
  private double kVel = 200;
  private double kAcel = 1000;
  private final TrapezoidProfile.Constraints m_Constraints =
      new TrapezoidProfile.Constraints(kAcel, kVel);
  private final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(kP, kI, kD, m_Constraints);
  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(2);

  @SuppressWarnings("rawtypes")
  private final LinearSystem elesys =
      LinearSystemId.createElevatorSystem(
          m_elevatorGearbox, Units.lbsToKilograms(5.5), Units.inchesToMeters(2.383), 6);

  @SuppressWarnings("unchecked")
  private final ElevatorSim m_ElevatorSim =
      new ElevatorSim(
          elesys, m_elevatorGearbox, 0, Units.inchesToMeters(55), true, Units.inchesToMeters(1));

  private final Encoder enc = new Encoder(4, 5);
  private final EncoderSim m_EncoderSim = new EncoderSim(enc);
  private final PWMTalonFX pwmTalonFX = new PWMTalonFX(10);
  // private final TalonFXSimState pwmTalonFX = new TalonFXSimState();
  private final PWMSim m_mototsim = new PWMSim(pwmTalonFX);
  public final LoggedMechanism2d m_mech2d =
      new LoggedMechanism2d(Units.inchesToMeters(28), Units.inchesToMeters(80));

  public final LoggedMechanismRoot2d m_mech2dRootSecondStage =
      m_mech2d.getRoot("Elevator Root 2", Units.inchesToMeters(19), Units.inchesToMeters(5.75));
  private final LoggedMechanismRoot2d m_mech2dRootFirstStage =
      m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(19), Units.inchesToMeters(4.75));

  public final LoggedMechanismLigament2d m_elevatorMechSecondStage2d =
      m_mech2dRootSecondStage.append(
          new LoggedMechanismLigament2d("SecondStageSim", m_ElevatorSim.getPositionMeters(), 90));
  private final LoggedMechanismLigament2d m_elevatorMechFirstStage2d =
      m_mech2dRootFirstStage.append(
          new LoggedMechanismLigament2d("FirstStageSim", m_ElevatorSim.getPositionMeters(), 90));

  public final LoggedMechanismLigament2d m_secondStage2d =
      m_elevatorMechSecondStage2d.append(
          new LoggedMechanismLigament2d(
              "SecondStage", Units.inchesToMeters(26.32), 0)); // Max height 27in
  private final LoggedMechanismLigament2d m_firstStage2d =
      m_elevatorMechFirstStage2d.append(
          new LoggedMechanismLigament2d(
              "FirstStage", Units.inchesToMeters(37), 0)); // Max height 28in

  /**
   * Constructs a new ElevatorIOSIM instance. Initializes the physics simulation with realistic
   * parameters including: - Dual Kraken X60 FOC motors - 10 pound carriage mass - 8 foot maximum
   * height - Gravity simulation enabled
   */
  public ElevatorIOSIM() {
    super();
    m_firstStage2d.setColor(new Color8Bit(Color.kNavy));
    m_secondStage2d.setColor(new Color8Bit(Color.kSteelBlue));
    enc.setDistancePerPulse((2 * Math.PI * Units.inchesToMeters(2.383)) / 4096);
    Logger.recordOutput("Elevator Sim", m_mech2d);
    SmartDashboard.putNumber("ElevatorSIM/PID/P", kP);
    SmartDashboard.putNumber("ElevatorSIM/PID/I", kI);
    SmartDashboard.putNumber("ElevatorSIM/PID/D", kD);
    SmartDashboard.putNumber("ElevatorSIM/PID/S", kS);
    SmartDashboard.putNumber("ElevatorSIM/PID/G", kG);
    SmartDashboard.putNumber("ElevatorSIM/PID/V", kV);
    SmartDashboard.putNumber("ElevatorSIM/PID/A", kA);
    SmartDashboard.putNumber("ElevatorSIM/PID/Acel", kAcel);
    SmartDashboard.putNumber("ElevatorSIM/PID/Vel", kVel);
  }

  /**
   * Updates the simulation model and all simulated sensor inputs.
   *
   * @param inputs The ElevatorIOInputs object to update with simulated values
   */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.elevatorDistance =
        Distance.ofRelativeUnits(enc.getDistance(), edu.wpi.first.units.Units.Inches);
    tempPIDTuning();
    m_elevatorMechSecondStage2d.setLength(
        enc.getDistance() > 0
            ? Units.inchesToMeters(enc.getDistance())
            : Units.inchesToMeters(0.000001));
    m_elevatorMechFirstStage2d.setLength(
        enc.getDistance() > 0.001
            ? (Units.inchesToMeters(enc.getDistance() * (28.0 / 55.0)))
            : Units.inchesToMeters(0.000001));
    m_ElevatorSim.setInput(m_mototsim.getSpeed() * RobotController.getBatteryVoltage());
    m_ElevatorSim.update(0.020);
    m_EncoderSim.setDistance(Units.metersToInches(m_ElevatorSim.getPositionMeters()));
    elevatorPID.setGoal(inputs.elevatorSetpoint.magnitude());
    if (inputs.manual != 0) {
      pwmTalonFX.set(inputs.manual);
    } else {
      pwmTalonFX.setVoltage(
          elevatorPID.calculate(enc.getDistance())
              + elevatorFF.calculate(elevatorPID.getSetpoint().velocity));
    }
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_ElevatorSim.getCurrentDrawAmps()));

    Logger.recordOutput("Elevator Sim", m_mech2d);
    Logger.recordOutput("ElevatorSIM/Position", enc.getDistance());
    Logger.recordOutput("ElevatorSIM/Goal", elevatorPID.getGoal().position);
    Logger.recordOutput("ElevatorSIM/Setpoint", elevatorPID.getSetpoint().position);
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
