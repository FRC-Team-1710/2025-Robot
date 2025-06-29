package frc.robot.subsystems.superstructure.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/**
 * This CoralIntake implementation is for a Talon FX driving a motor like the Falon 500 or Kraken
 * X60.
 */
public class ManipulatorIOCTRE implements ManipulatorIO {
  private final TalonFX manipulator = new TalonFX(21);
  private final DigitalInput breakingBeam1 = new DigitalInput(0);
  private final DigitalInput breakingBeam2 = new DigitalInput(1);
  final StatusSignal<Angle> positionRot = manipulator.getPosition();
  final StatusSignal<AngularVelocity> velocityRotPerSec = manipulator.getVelocity();
  final StatusSignal<Voltage> appliedVolts = manipulator.getMotorVoltage();
  final StatusSignal<Current> currentAmps = manipulator.getSupplyCurrent();

  public Timer timer = new Timer();

  public ManipulatorIOCTRE() {
    var config = new TalonFXConfiguration();
    config.Audio.AllowMusicDurDisable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> manipulator.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    manipulator.optimizeBusUtilization();
  }

  public void updateInputs(ManipulatorIOInputs ManipulatorIOInputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    ManipulatorIOInputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    ManipulatorIOInputs.velocityRadPerSec =
        Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    ManipulatorIOInputs.appliedVolts = appliedVolts.getValueAsDouble();
    ManipulatorIOInputs.currentAmps = currentAmps.getValueAsDouble();
    ManipulatorIOInputs.beam1Broken = !breakingBeam1.get();
    ManipulatorIOInputs.beam2Broken = !breakingBeam2.get();
  }

  @Override
  public void setVoltage(double volts) {
    manipulator.setVoltage(volts);
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }
}
