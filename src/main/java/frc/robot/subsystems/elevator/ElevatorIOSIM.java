package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOSIM extends ElevatorIOCTRE {
  private double kP = 0.5;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0.0;
  private double kG = 0.3564;
  private double kV = 3.18;
  private double kA = 0.0;
  private double kVel = 3;
  private double kAcel = 4.5;
  private boolean WasManuall = false;
  private final TrapezoidProfile.Constraints m_Constraints =
      new TrapezoidProfile.Constraints(kAcel, kVel);
  private final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(kP, kI, kD, m_Constraints);
  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(2);
  private final LinearSystem elesys =
      LinearSystemId.createElevatorSystem(
          m_elevatorGearbox, Units.lbsToKilograms(15), Units.inchesToMeters(1.5), 6);
  private final ElevatorSim m_ElevatorSim =
      new ElevatorSim(elesys, m_elevatorGearbox, 0, Units.inchesToMeters(55), true, 0.5);
  private final Encoder enc = new Encoder(3, 4);
  private final EncoderSim m_EncoderSim = new EncoderSim(enc);
  private final PWMTalonFX pwmTalonFX = new PWMTalonFX(0);
  private final PWMSim m_mototsim = new PWMSim(pwmTalonFX);
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  public final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_ElevatorSim.getPositionMeters(), 90));

  public ElevatorIOSIM() {
    super();
    enc.setDistancePerPulse((2 * Math.PI * Units.inchesToMeters(1.5)) / 4096);
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("S", kS);
    SmartDashboard.putNumber("G", kG);
    SmartDashboard.putNumber("V", kV);
    SmartDashboard.putNumber("A", kA);
    SmartDashboard.putNumber("Acel", kAcel);
    SmartDashboard.putNumber("Vel", kVel);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
    tempPIDTuning();
    m_elevatorMech2d.setLength(enc.getDistance());
    m_ElevatorSim.setInput(m_mototsim.getSpeed() * RobotController.getBatteryVoltage());
    m_ElevatorSim.update(0.020);
    m_EncoderSim.setDistance(m_ElevatorSim.getPositionMeters());

    elevatorPID.setGoal(inputs.setPoint.magnitude());

    if (inputs.manual != 0) {
      pwmTalonFX.setVoltage((inputs.manual * 12) + kG);
      WasManuall = true;
    } else {
      if (WasManuall) {
        elevatorPID.setGoal(enc.getDistance());
        WasManuall = false;
        pwmTalonFX.setVoltage(elevatorPID.calculate(enc.getDistance()) + elevatorFF.calculate(0));
      } else {
        pwmTalonFX.setVoltage(
            elevatorPID.calculate(enc.getDistance())
                + elevatorFF.calculate(elevatorPID.getSetpoint().velocity));
      }
    }
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_ElevatorSim.getCurrentDrawAmps()));
    SmartDashboard.putNumber("ElevatorSIM/Pos", enc.getDistance());
    SmartDashboard.putNumber("ElevatorSIM/Goal", elevatorPID.getGoal().position);
    SmartDashboard.putNumber("ElevatorSIM/Setpoint", elevatorPID.getSetpoint().position);
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
