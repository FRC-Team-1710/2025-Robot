package frc.robot.subsystems.manipulator;

import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This CoralIntake implementation is for a Talon FX driving a motor like the Falon 500 or Kraken
 * X60.
 */
public class ManipulatorIOTalonFX implements ManipulatorIO {
  private final TalonFX Manipulator = new TalonFX(21);
  private final DigitalInput BreakingBeam1 = new DigitalInput(0);
  private final DigitalInput BreakingBeam2 = new DigitalInput(1);
  final StatusSignal<Angle> positionRot = Manipulator.getPosition();
  final StatusSignal<AngularVelocity> velocityRotPerSec = Manipulator.getVelocity();
  final StatusSignal<Voltage> appliedVolts = Manipulator.getMotorVoltage();
  final StatusSignal<Current> currentAmps = Manipulator.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ManipulatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> Manipulator.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    Manipulator.optimizeBusUtilization();
  }

  public void updateInputs(ManipulatorIOInputs ManipulatorIOInputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    ManipulatorIOInputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    ManipulatorIOInputs.velocityRadPerSec =
        Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    ManipulatorIOInputs.appliedVolts = appliedVolts.getValueAsDouble();
    ManipulatorIOInputs.currentAmps = currentAmps.getValueAsDouble();
    ManipulatorIOInputs.beam1Broken = !BreakingBeam1.get();
    ManipulatorIOInputs.beam2Broken = !BreakingBeam2.get();
  }

  @Override
  public void setVoltage(double volts) {
    Manipulator.setControl(voltageRequest.withOutput(volts));
  }
}
