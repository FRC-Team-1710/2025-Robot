package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.elevator.ElevatorIOSIM;

public class ClawIOSIM extends ClawIOCTRE {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, 1), DCMotor.getCIM(1));
  private double appliedVolts = 0.0;
  private double sp = 0.0;
  private boolean locked = false;

  private double kp = 0.0;
  private double ki = 0.0;
  private double kd = 0.0;

  private final MechanismLigament2d m_wrist;
  private final DCMotor m_armGearbox = DCMotor.getKrakenX60(1);
  private final ProfiledPIDController m_bottomController =
      new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(5, 5));
  private final Encoder m_topEncoder = new Encoder(1, 2);
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
  private final SingleJointedArmSim m_arm_topSim =
      new SingleJointedArmSim(
          m_armGearbox,
          50,
          SingleJointedArmSim.estimateMOI(0.5, 0.5),
          0.5,
          Units.degreesToRadians(0),
          Units.degreesToRadians(170),
          true,
          Units.degreesToRadians(0));
  private final PWMTalonFX pwmTalonFX = new PWMTalonFX(1);
  private final EncoderSim m_topEncoderSim = new EncoderSim(m_topEncoder);

  public ClawIOSIM(ElevatorIOSIM elevator) {
    m_topEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    m_wrist =
        elevator.m_elevatorMech2d.append(
            new MechanismLigament2d("Wrist", 0.5, 0, 4, new Color8Bit(Color.kPurple)));
    SmartDashboard.putNumber("Claw/p", kp);
    SmartDashboard.putNumber("Claw/i", ki);
    SmartDashboard.putNumber("Claw/d", kd);
  }

  @Override
  public void updateInputs(ClawIOInputsAutoLogged inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.intakeVelocity = RotationsPerSecond.of(sim.getAngularVelocityRPM() * 60);
    inputs.intakeAppliedVoltage = Volts.of(appliedVolts);
    inputs.intakeStatorCurrent = Amps.of(sim.getCurrentDrawAmps());
    inputs.wristVelocity = RotationsPerSecond.of(m_topEncoderSim.getRate() * 60);
    inputs.wristAppliedVoltage = Volts.of(pwmTalonFX.getVoltage());
    inputs.setpoint = Degrees.of(sp);
    if (locked) {
      pwmTalonFX.setVoltage(
          m_bottomController.calculate(m_topEncoder.getDistance(), Units.degreesToRadians(sp)));
    }
    m_arm_topSim.setInput(pwmTalonFX.get() * RobotController.getBatteryVoltage());

    m_arm_topSim.update(0.02);

    m_wrist.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));

    m_wrist.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));

    if (kp != SmartDashboard.getNumber("Claw/p", kp)) {
      kp = SmartDashboard.getNumber("Claw/p", kp);
      m_bottomController.setP(kp);
    }

    if (ki != SmartDashboard.getNumber("Claw/i", ki)) {
      ki = SmartDashboard.getNumber("Claw/i", ki);
      m_bottomController.setI(ki);
    }

    if (kd != SmartDashboard.getNumber("Claw/d", kd)) {
      kd = SmartDashboard.getNumber("Claw/d", kd);
      m_bottomController.setD(kd);
    }
  }

  @Override
  public void setManual(double percent) {
    appliedVolts = MathUtil.clamp(percent, -12.0, 12.0);
  }

  @Override
  public void runPercent(double percent) {
    locked = false;
    pwmTalonFX.set(percent);
  }

  @Override
  public void setAngle(Angle angle) {
    sp = angle.magnitude();
  }
}
