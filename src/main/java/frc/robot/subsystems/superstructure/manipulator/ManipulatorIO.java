package frc.robot.subsystems.superstructure.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public interface ManipulatorIO {
  @Logged
  public class ManipulatorIOInputs {
    public Angle position = Rotations.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public double appliedVolts = 0.0;
    public double statorCurrent = 0.0;
    public boolean beam1Broken = false;
    public boolean beam2Broken = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ManipulatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
