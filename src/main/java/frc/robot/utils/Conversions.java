package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Conversions {
  public static Distance rotationsToMeters(
      Angle rotations, double gearRatio, Distance wheelRadius) {
    /* Apply gear ratio to input rotations */
    var gearedRadians = rotations.in(Radians) / gearRatio;
    /* Then multiply the wheel radius by radians of rotation to get distance */
    return wheelRadius.times(gearedRadians);
  }

  public static Angle metersToRotations(Distance meters, double gearRatio, Distance wheelRadius) {
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(Meters) / wheelRadius.in(Meters);
    /* Then multiply by gear ratio to get rotor rotations */
    return Radians.of(wheelRadians * gearRatio);
  }

  public static LinearVelocity rotationsToMetersVel(
      AngularVelocity rotations, double gearRatio, Distance wheelRadius) {
    /* Apply gear ratio to input rotations */
    var gearedRotations = rotations.in(RadiansPerSecond) / gearRatio;
    /* Then multiply the wheel radius by radians of rotation to get distance */
    return wheelRadius.per(Second).times(gearedRotations);
  }

  public static AngularVelocity metersToRotationsVel(
      LinearVelocity meters, double gearRatio, Distance wheelRadius) {
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(MetersPerSecond) / wheelRadius.in(Meters);
    /* Then multiply by gear ratio to get rotor rotations */
    return RadiansPerSecond.of(wheelRadians * gearRatio);
  }
}
