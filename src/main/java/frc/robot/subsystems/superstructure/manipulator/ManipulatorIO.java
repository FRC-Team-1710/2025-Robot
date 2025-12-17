package frc.robot.subsystems.superstructure.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public interface ManipulatorIO {
  @Logged
  public class ManipulatorIOInputs {
    @Logged(name = "Position", importance = Importance.INFO)
    public Angle position = Rotations.of(0);

    @Logged(name = "Velocity", importance = Importance.INFO)
    public AngularVelocity velocity = RotationsPerSecond.of(0);

    @Logged(name = "LeaderConnected", importance = Importance.INFO)
    public double appliedVolts = 0.0;

    @Logged(name = "StatorCurrent", importance = Importance.INFO)
    public double statorCurrent = 0.0;

    @Logged(name = "Beam1Broken", importance = Importance.INFO)
    public boolean beam1Broken = false;

    @Logged(name = "Beam2Broken", importance = Importance.INFO)
    public boolean beam2Broken = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ManipulatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
