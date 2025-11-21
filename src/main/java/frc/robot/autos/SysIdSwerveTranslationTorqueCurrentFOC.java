package frc.robot.autos;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Current;

public class SysIdSwerveTranslationTorqueCurrentFOC implements SwerveRequest {
  private double ampsToApply = 0.0;

  /** Local reference to a voltage request for the drive motors */
  private final TorqueCurrentFOC driveRequest = new TorqueCurrentFOC(0.0);

  /** Local reference to a position voltage request for the steer motors */
  private final PositionVoltage steerRequestVoltage = new PositionVoltage(0.0);

  /** Local reference to a position torque current request for the steer motors */
  private final PositionTorqueCurrentFOC steerRequestTorqueCurrent =
      new PositionTorqueCurrentFOC(0.0);

  @Override
  public StatusCode apply(
      SwerveDrivetrain.SwerveControlParameters parameters,
      SwerveModule<?, ?, ?>... modulesToApply) {

    for (SwerveModule<?, ?, ?> module : modulesToApply) {
      switch (module.getSteerClosedLoopOutputType()) {
        case Voltage:
          module.apply(driveRequest.withOutput(ampsToApply), steerRequestVoltage.withPosition(0.0));
          break;
        case TorqueCurrentFOC:
          module.apply(
              driveRequest.withOutput(ampsToApply), steerRequestTorqueCurrent.withPosition(0.0));
          break;
      }
    }
    return StatusCode.OK;
  }

  /**
   * Sets the current to apply to the drive wheels.
   *
   * @param amps Current to apply
   * @return this request
   */
  public SysIdSwerveTranslationTorqueCurrentFOC withCurrent(double amps) {
    this.ampsToApply = amps;
    return this;
  }

  /**
   * Sets the current to apply to the drive wheels.
   *
   * @param amps Current to apply
   * @return this request
   */
  public SysIdSwerveTranslationTorqueCurrentFOC withCurrent(Current amps) {
    this.ampsToApply = amps.in(Amps);
    return this;
  }
}
