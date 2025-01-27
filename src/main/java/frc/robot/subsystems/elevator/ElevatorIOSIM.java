package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

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
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kVel = 0.0;
  private double kAcel = 0.0;
  private boolean WasManuall = false;
  private final TrapezoidProfile.Constraints m_Constraints =
      new TrapezoidProfile.Constraints(kAcel, kVel);
  private final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(kP, kI, kD, m_Constraints);
  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(2);
  private final LinearSystem elesys =
      LinearSystemId.createElevatorSystem(
          m_elevatorGearbox, Units.lbsToKilograms(15), Units.inchesToMeters(2.383), 6);
  private final ElevatorSim m_ElevatorSim =
      new ElevatorSim(elesys, m_elevatorGearbox, 0, Units.inchesToMeters(55), true, 0.5);
  private final Encoder enc = new Encoder(3, 4);
  private final EncoderSim m_EncoderSim = new EncoderSim(enc);
  private final PWMTalonFX pwmTalonFX = new PWMTalonFX(0);
  private final PWMSim m_mototsim = new PWMSim(pwmTalonFX);
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("ElevatorSimulator", m_ElevatorSim.getPositionMeters(), 90));

  public ElevatorIOSIM() {
    super();
    enc.setDistancePerPulse((2 * Math.PI * Units.inchesToMeters(2.383)) / 4096);
    SmartDashboard.putData("Elevator Sim", m_mech2d);
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

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
    tempPIDTuning();
    m_elevatorMech2d.setLength(enc.getDistance());
    m_ElevatorSim.setInput(m_mototsim.getSpeed() * RobotController.getBatteryVoltage());
    m_ElevatorSim.update(0.020);
    m_EncoderSim.setDistance(m_ElevatorSim.getPositionMeters());
    if (inputs.SIMsetpoint == null) {
      elevatorPID.setGoal(inputs.setpoint.magnitude());
    } else {
      elevatorPID.setGoal(inputs.SIMsetpoint.magnitude());
    }
    if (inputs.manualSpin != 0) {
      pwmTalonFX.setVoltage(Math.pow(inputs.manualSpin * 1.5, 3) + kG);
      WasManuall = true;
    } else {
      if (WasManuall) {
        ElevatorIOInputs.SIMsetpoint = Meters.of(enc.getDistance());
        elevatorPID.reset(enc.getDistance());
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
    SmartDashboard.putNumber("ElevatorSIM/Position", enc.getDistance());
    SmartDashboard.putNumber("ElevatorSIM/Goal", elevatorPID.getGoal().position);
    SmartDashboard.putNumber("ElevatorSIM/Setpoint", elevatorPID.getSetpoint().position);
  }

  private void tempPIDTuning() {
    if (kP != SmartDashboard.getNumber("ElevatorSIM/PID/P", kP)) {
      kP = SmartDashboard.getNumber("ElevatorSIM/PID/P", kP);
      elevatorPID.setP(kP);
    }

    if (kI != SmartDashboard.getNumber("ElevatorSIM/PID/I", kI)) {
      kI = SmartDashboard.getNumber("ElevatorSIM/PID/I", kI);
      elevatorPID.setI(kI);
    }

    if (kD != SmartDashboard.getNumber("ElevatorSIM/PID/D", kD)) {
      kD = SmartDashboard.getNumber("ElevatorSIM/PID/D", kD);
      elevatorPID.setD(kD);
    }

    if (kS != SmartDashboard.getNumber("ElevatorSIM/PID/S", kS)) {
      kS = SmartDashboard.getNumber("ElevatorSIM/PID/S", kS);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kG != SmartDashboard.getNumber("ElevatorSIM/PID/G", kG)) {
      kG = SmartDashboard.getNumber("ElevatorSIM/PID/G", kG);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kV != SmartDashboard.getNumber("ElevatorSIM/PID/V", kV)) {
      kV = SmartDashboard.getNumber("ElevatorSIM/PID/V", kV);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kA != SmartDashboard.getNumber("ElevatorSIM/PID/A", kA)) {
      kA = SmartDashboard.getNumber("ElevatorSIM/PID/A", kA);
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    if (kAcel != SmartDashboard.getNumber("ElevatorSIM/PID/Acel", kAcel)) {
      kAcel = SmartDashboard.getNumber("ElevatorSIM/PID/Acel", kAcel);
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }

    if (kVel != SmartDashboard.getNumber("ElevatorSIM/PID/Vel", kVel)) {
      kVel = SmartDashboard.getNumber("ElevatorSIM/PID/Vel", kVel);
      elevatorPID.setConstraints(new TrapezoidProfile.Constraints(kAcel, kVel));
    }
  }
}
