package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClawIOCTRE implements ClawIO {
  public static final double GEAR_RATIO = 66.6666666;
  private boolean locked = false;
  private boolean rollerLocked = false;
  private boolean hasZeroed = false;

  private double kP = 1.5;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kAcel = 65;
  private double kVel = 85;

  private double RollerkP = 3;
  private double RollerkI = 0.0;
  private double RollerkD = 0.0;

  public final TalonFX wrist = new TalonFX(51);
  public final TalonFX rollers = new TalonFX(52);

  private final PIDController rollerPID = new PIDController(RollerkP, RollerkI, RollerkD);
  // private final ProfiledPIDController wristPID =
  //     new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kvel, kacel));
  // private final ArmFeedforward wristFF = new ArmFeedforward(kS, kG, kV, kA);

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> wristPosition = wrist.getPosition();
  private final StatusSignal<Double> wristRefrence = wrist.getClosedLoopReference();
  private final StatusSignal<AngularVelocity> wristVelocity = wrist.getVelocity();
  private final StatusSignal<Voltage> wristAppliedVolts = wrist.getMotorVoltage();
  private final StatusSignal<Current> wristStatorCurrent = wrist.getStatorCurrent();
  private final StatusSignal<Current> wristSupplyCurrent = wrist.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> intakeVelocity = rollers.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = rollers.getMotorVoltage();
  private final StatusSignal<Current> intakeStatorCurrent = rollers.getStatorCurrent();
  private final StatusSignal<Current> intakeSupplyCurrent = rollers.getSupplyCurrent();

  private final Debouncer clawDebounce = new Debouncer(0.5);
  private final Debouncer wristDebounce = new Debouncer(0.5);

  private Angle setAngle = Degrees.of(0);
  private double wristManual = 0.0;
  private double runPercent = 0.0;

  TalonFXConfiguration config2 = new TalonFXConfiguration();

  public ClawIOCTRE() {

    if (Constants.useSmartDashboard) {
      SmartDashboard.putBoolean("Claw/PID/OMG", false);
      SmartDashboard.putNumber("Claw/PID/P", kP);
      SmartDashboard.putNumber("Claw/PID/I", kI);
      SmartDashboard.putNumber("Claw/PID/D", kD);
      SmartDashboard.putNumber("Claw/PID/S", kS);
      SmartDashboard.putNumber("Claw/PID/G", kG);
      SmartDashboard.putNumber("Claw/PID/V", kV);
      SmartDashboard.putNumber("Claw/PID/A", kA);
      SmartDashboard.putNumber("Claw/PID/Vel", kVel);
      SmartDashboard.putNumber("Claw/PID/Acel", kAcel);

      SmartDashboard.putNumber("Claw/RollerPID/P", RollerkP);
      SmartDashboard.putNumber("Claw/RollerPID/I", RollerkI);
      SmartDashboard.putNumber("Claw/RollerPID/D", RollerkD);
    }

    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wrist.getConfigurator().apply(createMotorConfiguration());
    rollers.getConfigurator().apply(config2);

    wrist.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristPosition,
        wristRefrence,
        wristVelocity,
        wristStatorCurrent,
        wristSupplyCurrent,
        intakeVelocity,
        intakeAppliedVolts,
        intakeStatorCurrent,
        intakeSupplyCurrent);
  }

  private TalonFXConfiguration createMotorConfiguration() {
    var config = new TalonFXConfiguration();
    if (Constants.useSmartDashboard) {
      config.Slot0.kP = SmartDashboard.getNumber("Claw/PID/P", kP);
      config.Slot0.kI = SmartDashboard.getNumber("Claw/PID/I", kI);
      config.Slot0.kD = SmartDashboard.getNumber("Claw/PID/D", kD);
      config.Slot0.kS = SmartDashboard.getNumber("Claw/PID/S", kS);
      config.Slot0.kG = SmartDashboard.getNumber("Claw/PID/G", kG);
      config.Slot0.kV = SmartDashboard.getNumber("Claw/PID/V", kV);
      config.Slot0.kA = SmartDashboard.getNumber("Claw/PID/A", kA);
      config.MotionMagic.MotionMagicAcceleration = SmartDashboard.getNumber("Claw/PID/Acel", kAcel);
      config.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("Claw/PID/Vel", kVel);
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
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return config;
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    StatusCode wristStatus =
        BaseStatusSignal.refreshAll(
            wristPosition,
            wristRefrence,
            wristVelocity,
            wristAppliedVolts,
            wristStatorCurrent,
            wristSupplyCurrent);

    StatusCode clawStatus =
        BaseStatusSignal.refreshAll(
            intakeVelocity, intakeAppliedVolts, intakeStatorCurrent, intakeSupplyCurrent);

    inputs.clawConnected = clawDebounce.calculate(clawStatus.isOK());
    inputs.wristConnected = wristDebounce.calculate(wristStatus.isOK());

    inputs.wristMotorAngle = Degrees.of(wristPosition.getValue().magnitude() * 360 / GEAR_RATIO);

    inputs.wristVelocity =
        DegreesPerSecond.of(wristVelocity.getValue().magnitude() * 360 / GEAR_RATIO);
    inputs.intakeVelocity =
        DegreesPerSecond.of(intakeVelocity.getValue().magnitude() * 360 / GEAR_RATIO);

    inputs.locked = locked;

    inputs.wristAppliedVoltage = wristAppliedVolts.getValue();
    inputs.wristStatorCurrent = wristStatorCurrent.getValue();
    inputs.wristSupplyCurrent = wristSupplyCurrent.getValue();
    inputs.rollerAppliedVoltage = intakeAppliedVolts.getValue();
    inputs.rollerStatorCurrent = intakeStatorCurrent.getValue();
    inputs.rollerSupplyCurrent = intakeSupplyCurrent.getValue();
    inputs.rollerPosition = rollers.getPosition().getValueAsDouble();

    inputs.setpoint = setAngle;
    inputs.wristManual = wristManual;
    inputs.intakePercent = runPercent;

    inputs.rollerLocked = rollerLocked;
    inputs.angle = Degrees.of((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO));

    inputs.hasZeroed = hasZeroed;

    if (Constants.useSmartDashboard) {
      tempPIDTuning();

      SmartDashboard.putNumber("Claw Inches", wrist.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Claw Setpoint", wrist.getClosedLoopReference().getValueAsDouble());
    }

    // if (locked) {
    //   if (inputs.killSwich) {
    //     wrist.stopMotor();
    //   } else {
    //     wrist.setVoltage(
    //         wristPID.calculate(inputs.angle.magnitude())
    //             + wristFF.calculate(inputs.angle.in(Radians), wristPID.getSetpoint().velocity));
    //   }
    // }

    if (rollerLocked) {
      rollers.setVoltage(rollerPID.calculate(inputs.rollerPosition));
    }

    Logger.recordOutput("roller locked", rollerLocked);
  }

  @Override
  public void setAngle(Angle angle) {
    setAngle = angle;
    wrist.setControl(request.withPosition(angle.times(-GEAR_RATIO)));
    locked = true;
  }

  @Override
  public void zero() {
    // wrist.setPosition(0);
    // wristPID.reset(0);
    // hasZeroed = true;
  }

  @Override
  public void setBrake(boolean lock) {
    // config.MotorOutput.NeutralMode = lock ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    // rollers.getConfigurator().apply(config);
  }

  @Override
  public void lockRoller() {
    rollerPID.setSetpoint(rollers.getPosition().getValueAsDouble());
    rollerLocked = true;
  }

  @Override
  public void stopHere() {
    // wristPID.reset(((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO)), 0);
    // setAngle = Degrees.of((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO));
    // locked = true;
  }

  @Override
  public void wristManual(double power) {
    // locked = false;
    // wristManual = power;
    // wrist.setVoltage(wristManual);
  }

  @Override
  public void setRollers(double power) {
    runPercent = power;
    rollerLocked = false;
    rollers.set(power);
  }

  @Override
  public void zeroPIDToAngle() {
    // wristPID.reset((wristPosition.getValueAsDouble() * 360 / GEAR_RATIO), 0);
  }

  private void tempPIDTuning() {
    if (SmartDashboard.getBoolean("Claw/PID/OMG", false)) {
      SmartDashboard.putBoolean("Claw/PID/OMG", false);
      wrist.getConfigurator().apply(createMotorConfiguration());
    }

    if (RollerkP != SmartDashboard.getNumber("Claw/RollerPID/P", RollerkP)) {
      RollerkP = SmartDashboard.getNumber("Claw/RollerPID/P", RollerkP);
      rollerPID.setP(RollerkP);
    }

    if (RollerkI != SmartDashboard.getNumber("Claw/RollerPID/I", RollerkI)) {
      RollerkI = SmartDashboard.getNumber("Claw/RollerPID/I", RollerkI);
      rollerPID.setI(RollerkI);
    }

    if (RollerkD != SmartDashboard.getNumber("Claw/RollerPID/D", RollerkD)) {
      RollerkD = SmartDashboard.getNumber("Claw/RollerPID/D", RollerkD);
      rollerPID.setD(RollerkD);
    }
  }
}
