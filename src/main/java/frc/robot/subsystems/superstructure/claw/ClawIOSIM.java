package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSIM;

@Logged
public class ClawIOSIM extends ClawIOCTRE {
  @NotLogged
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, 1),
          DCMotor.getKrakenX60(1));

  @NotLogged private double appliedVolts = 0.0;
  @NotLogged private Angle sp = Degrees.of(0.0);

  @Logged(name = "kp", importance = Importance.DEBUG)
  private double kp = 0.1;

  @Logged(name = "ki", importance = Importance.DEBUG)
  private double ki = 0.0;

  @Logged(name = "kd", importance = Importance.DEBUG)
  private double kd = 0.0175;

  @Logged(name = "ks", importance = Importance.DEBUG)
  private double ks = 0.0;

  @Logged(name = "kg", importance = Importance.DEBUG)
  private double kg = 0.035100;

  @Logged(name = "kv", importance = Importance.DEBUG)
  private double kv = 0.019;

  @Logged(name = "ka", importance = Importance.DEBUG)
  private double ka = 0.0;

  @Logged(name = "maxacel", importance = Importance.DEBUG)
  private double maxacel = 500.0;

  @Logged(name = "maxvel", importance = Importance.DEBUG)
  private double maxvel = 200.0;

  @NotLogged private final MechanismLigament2d m_wrist;
  @NotLogged private final MechanismLigament2d m_wrist2;
  @NotLogged private final MechanismLigament2d m_wristEXTENSION;
  @NotLogged private final MechanismLigament2d m_wristEXTENSION2;

  @NotLogged private final MechanismLigament2d[] m_Algae;

  @NotLogged private final DCMotor m_armGearbox = DCMotor.getKrakenX60(1);

  @NotLogged
  private final ProfiledPIDController m_bottomController =
      new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(maxvel, maxacel));

  @NotLogged private ArmFeedforward wristff = new ArmFeedforward(ks, kg, kv, ka);
  @NotLogged private final Encoder m_topEncoder = new Encoder(11, 12);
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  @NotLogged
  private final SingleJointedArmSim m_arm_topSim =
      new SingleJointedArmSim(
          m_armGearbox,
          50,
          SingleJointedArmSim.estimateMOI(0.5, 0.5),
          0.5,
          Units.degreesToRadians(280),
          Units.degreesToRadians(450),
          true,
          Units.degreesToRadians(0));

  @NotLogged private final PWMTalonFX pwmTalonFX = new PWMTalonFX(1);
  @NotLogged private final EncoderSim m_topEncoderSim = new EncoderSim(m_topEncoder);

  public ClawIOSIM(ElevatorIOSIM elevator) {
    m_topEncoderSim.setDistancePerPulse(kArmEncoderDistPerPulse);
    m_wristEXTENSION =
        elevator.m_secondStage2d.append(
            new MechanismLigament2d("Extension", Units.inchesToMeters(9.643), 230));
    m_wristEXTENSION2 =
        m_wristEXTENSION.append(
            new MechanismLigament2d("Extension2", Units.inchesToMeters(8.25), -90));
    m_wrist =
        m_wristEXTENSION2.append(
            new MechanismLigament2d(
                "Wrist", Units.inchesToMeters(8.729), 0, 4.8, new Color8Bit(Color.kPurple)));
    m_wrist2 =
        m_wrist.append(
            new MechanismLigament2d(
                "Wrist2", Units.inchesToMeters(12.625), 26.6, 4.8, new Color8Bit(Color.kPurple)));
    m_Algae = new MechanismLigament2d[15];
    // for (int i = 0; i < 15; i++) {
    //   m_Algae[i] =
    //       m_wrist.append(
    //           new LoggedMechanismLigament2d(
    //               "Algae Line " + i,
    //               Units.inchesToMeters(16.5) / 2.0,
    //               (360.0 / 15) * i,
    //               16.5,
    //               new Color8Bit(Color.kAquamarine)));
    // }
    // if (Constants.useSmartDashboard) {
    //   SmartDashboard.putNumber("Claw/p", kp);
    //   SmartDashboard.putNumber("Claw/i", ki);
    //   SmartDashboard.putNumber("Claw/d", kd);
    //   SmartDashboard.putNumber("Claw/s", ks);
    //   SmartDashboard.putNumber("Claw/g", kg);
    //   SmartDashboard.putNumber("Claw/v", kv);
    //   SmartDashboard.putNumber("Claw/a", ka);
    //   SmartDashboard.putNumber("Claw/maxacel", maxacel);
    //   SmartDashboard.putNumber("Claw/maxvel", maxvel);
    // }
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.angle =
        Angle.ofRelativeUnits(
            Units.degreesToRadians(90) - (m_arm_topSim.getAngleRads() - (2 * Math.PI)), Radian);
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    sp = inputs.setpoint;

    if (inputs.wristManual == 0) {
      pwmTalonFX.setVoltage(
          m_bottomController.calculate(
                  Units.radiansToDegrees(m_arm_topSim.getAngleRads()), 170 - sp.magnitude() + 280)
              + wristff.calculate(
                  m_arm_topSim.getAngleRads(), m_bottomController.getSetpoint().velocity));
      // if (Constants.useSmartDashboard) {
      //   SmartDashboard.putNumber("position",
      // Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
      //   SmartDashboard.putNumber("setpoint", 170 - sp.magnitude() + 280);
      //   SmartDashboard.putNumber("goal", m_bottomController.getSetpoint().position);
      // }
    } else {
      pwmTalonFX.set(inputs.wristManual);
    }
    m_arm_topSim.setInput(pwmTalonFX.get() * RobotController.getBatteryVoltage());

    m_wrist.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()) + 40 + 90 - 26.6);

    m_arm_topSim.update(0.02);

    // if (Constants.useSmartDashboard) {
    //   if (kp != SmartDashboard.getNumber("Claw/p", kp)) {
    //     kp = SmartDashboard.getNumber("Claw/p", kp);
    //     m_bottomController.setP(kp);
    //   }

    //   if (ki != SmartDashboard.getNumber("Claw/i", ki)) {
    //     ki = SmartDashboard.getNumber("Claw/i", ki);
    //     m_bottomController.setI(ki);
    //   }

    //   if (kd != SmartDashboard.getNumber("Claw/d", kd)) {
    //     kd = SmartDashboard.getNumber("Claw/d", kd);
    //     m_bottomController.setD(kd);
    //   }

    //   if (ks != SmartDashboard.getNumber("Claw/s", ks)) {
    //     ks = SmartDashboard.getNumber("Claw/s", ks);
    //     wristff = new ArmFeedforward(ks, kg, kv, ka);
    //   }

    //   if (kg != SmartDashboard.getNumber("Claw/g", kg)) {
    //     kg = SmartDashboard.getNumber("Claw/g", kg);
    //     wristff = new ArmFeedforward(ks, kg, kv, ka);
    //   }

    //   if (kv != SmartDashboard.getNumber("Claw/v", kv)) {
    //     kv = SmartDashboard.getNumber("Claw/v", kv);
    //     wristff = new ArmFeedforward(ks, kg, kv, ka);
    //   }

    //   if (ka != SmartDashboard.getNumber("Claw/a", ka)) {
    //     ka = SmartDashboard.getNumber("Claw/a", ka);
    //     wristff = new ArmFeedforward(ks, kg, kv, ka);
    //   }

    //   if (maxacel != SmartDashboard.getNumber("Claw/maxacel", maxacel)) {
    //     maxacel = SmartDashboard.getNumber("Claw/maxacel", maxacel);
    //     m_bottomController.setConstraints(new TrapezoidProfile.Constraints(maxvel, maxacel));
    //   }

    //   if (maxvel != SmartDashboard.getNumber("Claw/maxvel", maxvel)) {
    //     maxvel = SmartDashboard.getNumber("Claw/maxvel", maxvel);
    //     m_bottomController.setConstraints(new TrapezoidProfile.Constraints(maxvel, maxacel));
    //   }
    // }

    if (inputs.hasAlgae) {
      m_wrist2.setColor(new Color8Bit(Color.kAquamarine));
    } else {
      m_wrist2.setColor(new Color8Bit(Color.kPurple));
    }
  }
}
