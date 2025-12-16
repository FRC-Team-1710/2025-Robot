package frc.robot.subsystems.superstructure.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

/**
 * This CoralIntake implementation is for a Talon FX driving a motor like the Falcon 500 or Kraken
 * X60.
 */
@Logged
public class ManipulatorIOCTRE implements ManipulatorIO {
  @Logged(name = "Motor", importance = Importance.INFO)
  private final TalonFX manipulator = new TalonFX(21);
  @Logged(name = "Beam1", importance = Importance.CRITICAL)
  private final DigitalInput breakingBeam1 = new DigitalInput(0);
  @Logged(name = "Beam2", importance = Importance.CRITICAL)
  private final DigitalInput breakingBeam2 = new DigitalInput(1);
  @NotLogged
  final StatusSignal<Angle> position = manipulator.getPosition();
  @NotLogged
  final StatusSignal<AngularVelocity> velocity = manipulator.getVelocity();
  @NotLogged
  final StatusSignal<Voltage> appliedVolts = manipulator.getMotorVoltage();
  @NotLogged
  final StatusSignal<Current> current = manipulator.getSupplyCurrent();

  public ManipulatorIOCTRE() {
    var config = new TalonFXConfiguration();
    config.Audio.AllowMusicDurDisable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    manipulator.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current);
    manipulator.optimizeBusUtilization();
  }

  public void updateInputs(ManipulatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrent = current.getValueAsDouble();
    inputs.beam1Broken = !breakingBeam1.get();
    inputs.beam2Broken = !breakingBeam2.get();
  }

  @Override
  public void setVoltage(double volts) {
    manipulator.setVoltage(volts);
  }
}
