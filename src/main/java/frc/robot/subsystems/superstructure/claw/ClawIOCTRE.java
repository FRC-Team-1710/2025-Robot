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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class ClawIOCTRE implements ClawIO {
  public static final double GEAR_RATIO = 66.6666666;

  @Logged(name = "Locked", importance = Importance.INFO)
  private boolean locked = false;

  @Logged(name = "RollerLocked", importance = Importance.INFO)
  private boolean rollerLocked = false;

  @Logged(name = "HasZeroed", importance = Importance.INFO)
  private boolean hasZeroed = false;

  @Logged(name = "kP", importance = Importance.INFO)
  private double kP = 1.5;

  @Logged(name = "kI", importance = Importance.INFO)
  private double kI = 0.0;

  @Logged(name = "kD", importance = Importance.INFO)
  private double kD = 0.0;

  @Logged(name = "kS", importance = Importance.INFO)
  private double kS = 0.0;

  @Logged(name = "kG", importance = Importance.INFO)
  private double kG = 0.0;

  @Logged(name = "kV", importance = Importance.INFO)
  private double kV = 0.0;

  @Logged(name = "kA", importance = Importance.INFO)
  private double kA = 0.0;

  @Logged(name = "kAcel", importance = Importance.INFO)
  private double kAcel = 65;

  @Logged(name = "kVel", importance = Importance.INFO)
  private double kVel = 85;

  @Logged(name = "RollerKP", importance = Importance.INFO)
  private double rollerKP = 3;

  @Logged(name = "RollerKI", importance = Importance.INFO)
  private double rollerKI = 0.0;

  @Logged(name = "RollerKD", importance = Importance.INFO)
  private double rollerKD = 0.0;

  @Logged(name = "Wrist", importance = Importance.CRITICAL)
  public final TalonFX wrist = new TalonFX(51);

  @Logged(name = "Rollers", importance = Importance.CRITICAL)
  public final TalonFX rollers = new TalonFX(52);

  @NotLogged
  private final PIDController rollerPID = new PIDController(rollerKP, rollerKI, rollerKD);

  // private final ProfiledPIDController wristPID =
  //     new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kVel, kAcel));
  // private final ArmFeedforward wristFF = new ArmFeedforward(kS, kG, kV, kA);

  @NotLogged private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

  @NotLogged private final StatusSignal<Angle> wristPosition = wrist.getPosition();
  @NotLogged private final StatusSignal<Double> wristReference = wrist.getClosedLoopReference();
  @NotLogged private final StatusSignal<AngularVelocity> wristVelocity = wrist.getVelocity();
  @NotLogged private final StatusSignal<Voltage> wristAppliedVolts = wrist.getMotorVoltage();
  @NotLogged private final StatusSignal<Current> wristStatorCurrent = wrist.getStatorCurrent();
  @NotLogged private final StatusSignal<Current> wristSupplyCurrent = wrist.getSupplyCurrent();
  @NotLogged private final StatusSignal<AngularVelocity> intakeVelocity = rollers.getVelocity();
  @NotLogged private final StatusSignal<Voltage> intakeAppliedVolts = rollers.getMotorVoltage();
  @NotLogged private final StatusSignal<Current> intakeStatorCurrent = rollers.getStatorCurrent();
  @NotLogged private final StatusSignal<Current> intakeSupplyCurrent = rollers.getSupplyCurrent();

  @NotLogged private final Debouncer clawDebounce = new Debouncer(0.5);
  @NotLogged private final Debouncer wristDebounce = new Debouncer(0.5);

  @Logged(name = "SetAngle", importance = Importance.INFO)
  private Angle setAngle = Degrees.of(0);

  @Logged(name = "WristManual", importance = Importance.INFO)
  private double wristManual = 0.0;

  @Logged(name = "RunPercent", importance = Importance.INFO)
  private double runPercent = 0.0;

  @NotLogged TalonFXConfiguration config2 = new TalonFXConfiguration();

  public ClawIOCTRE() {

    // if (Constants.useSmartDashboard) {
    //   SmartDashboard.putBoolean("Claw/PID/OMG", false);
    //   SmartDashboard.putNumber("Claw/PID/P", kP);
    //   SmartDashboard.putNumber("Claw/PID/I", kI);
    //   SmartDashboard.putNumber("Claw/PID/D", kD);
    //   SmartDashboard.putNumber("Claw/PID/S", kS);
    //   SmartDashboard.putNumber("Claw/PID/G", kG);
    //   SmartDashboard.putNumber("Claw/PID/V", kV);
    //   SmartDashboard.putNumber("Claw/PID/A", kA);
    //   SmartDashboard.putNumber("Claw/PID/Vel", kVel);
    //   SmartDashboard.putNumber("Claw/PID/Acel", kAcel);

    //   SmartDashboard.putNumber("Claw/RollerPID/P", rollerKP);
    //   SmartDashboard.putNumber("Claw/RollerPID/I", rollerKI);
    //   SmartDashboard.putNumber("Claw/RollerPID/D", rollerKD);
    // }

    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wrist.getConfigurator().apply(createMotorConfiguration());
    rollers.getConfigurator().apply(config2);

    wrist.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristPosition,
        wristReference,
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
    // if (Constants.useSmartDashboard) {
    //   config.Slot0.kP = SmartDashboard.getNumber("Claw/PID/P", kP);
    //   config.Slot0.kI = SmartDashboard.getNumber("Claw/PID/I", kI);
    //   config.Slot0.kD = SmartDashboard.getNumber("Claw/PID/D", kD);
    //   config.Slot0.kS = SmartDashboard.getNumber("Claw/PID/S", kS);
    //   config.Slot0.kG = SmartDashboard.getNumber("Claw/PID/G", kG);
    //   config.Slot0.kV = SmartDashboard.getNumber("Claw/PID/V", kV);
    //   config.Slot0.kA = SmartDashboard.getNumber("Claw/PID/A", kA);
    //   config.MotionMagic.MotionMagicAcceleration = SmartDashboard.getNumber("Claw/PID/Acel",
    // kAcel);
    //   config.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("Claw/PID/Vel",
    // kVel);
    // } else {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.MotionMagic.MotionMagicAcceleration = kAcel;
    config.MotionMagic.MotionMagicCruiseVelocity = kVel;
    // }
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return config;
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    StatusCode wristStatus =
        BaseStatusSignal.refreshAll(
            wristPosition,
            wristReference,
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

    // if (Constants.useSmartDashboard) {
    //   tempPIDTuning();

    //   SmartDashboard.putNumber("Claw Inches", wrist.getPosition().getValueAsDouble());
    //   SmartDashboard.putNumber("Claw Setpoint",
    // wrist.getClosedLoopReference().getValueAsDouble());
    // }

    // if (locked) {
    //   if (inputs.killSwitch) {
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

  // private void tempPIDTuning() {
  //   if (SmartDashboard.getBoolean("Claw/PID/OMG", false)) {
  //     SmartDashboard.putBoolean("Claw/PID/OMG", false);
  //     wrist.getConfigurator().apply(createMotorConfiguration());
  //   }

  //   if (rollerKP != SmartDashboard.getNumber("Claw/RollerPID/P", rollerKP)) {
  //     rollerKP = SmartDashboard.getNumber("Claw/RollerPID/P", rollerKP);
  //     rollerPID.setP(rollerKP);
  //   }

  //   if (rollerKI != SmartDashboard.getNumber("Claw/RollerPID/I", rollerKI)) {
  //     rollerKI = SmartDashboard.getNumber("Claw/RollerPID/I", rollerKI);
  //     rollerPID.setI(rollerKI);
  //   }

  //   if (rollerKD != SmartDashboard.getNumber("Claw/RollerPID/D", rollerKD)) {
  //     rollerKD = SmartDashboard.getNumber("Claw/RollerPID/D", rollerKD);
  //     rollerPID.setD(rollerKD);
  //   }
  // }
}
