/*
 * Copyright (C) Cross The Road Electronics. All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;

/**
 * Calculates wheel force feedforwards for a {@link SwerveDrivetrain}. Wheel force feedforwards can
 * be used to improve accuracy of path following in regions of acceleration.
 *
 * <p>Many path planning libraries (such as PathPlanner and Choreo) already provide per-module force
 * feedforwards, which should be preferred over this class.
 */
public class WheelForceCalculator {
  /** Wheel force feedforwards to apply to a swerve drivetrain. */
  public static class Feedforwards {
    /** The X component of the forces in Newtons. */
    public final double[] x_newtons;

    /** The Y component of the forces in Newtons. */
    public final double[] y_newtons;

    /**
     * Constructs a new set of wheel force feedforwards.
     *
     * @param numModules The number of swerve modules
     */
    public Feedforwards(int numModules) {
      x_newtons = new double[numModules];
      y_newtons = new double[numModules];
    }
  }

  private final Translation2d[] moduleLocations;
  private final double mass;
  private final double moi;

  /**
   * Constructs a new wheel force feedforward calculator for the given drivetrain characteristics.
   *
   * @param moduleLocations The drivetrain swerve module locations
   * @param mass_kg The mass of the robot in kg
   * @param moi_kg_m_sq The moment of inertia of the robot in kg m^2
   */
  public WheelForceCalculator(Translation2d[] moduleLocations, double mass_kg, double moi_kg_m_sq) {
    this.moduleLocations = moduleLocations;
    this.mass = mass_kg;
    this.moi = moi_kg_m_sq;
  }

  /**
   * Constructs a new wheel force feedforward calculator for the given drivetrain characteristics.
   *
   * @param moduleLocations The drivetrain swerve module locations
   * @param mass The mass of the robot
   * @param moi The moment of inertia of the robot
   */
  public WheelForceCalculator(Translation2d[] moduleLocations, Mass mass, MomentOfInertia moi) {
    this(moduleLocations, mass.in(Kilograms), moi.in(KilogramSquareMeters));
  }

  /**
   * Calculates wheel force feedforwards for the desired robot acceleration. This can be used with
   * either robot-centric or field-centric accelerations; the returned force feedforwards will match
   * in being robot-/field-centric.
   *
   * @param ax Acceleration in the X direction (forward) in m/s^2
   * @param ay Acceleration in the Y direction (left) in m/s^2
   * @param alpha Angular acceleration in rad/s^2
   * @return Wheel force feedforwards to apply
   */
  public final Feedforwards calculate(double ax, double ay, double alpha) {
    return calculate(ax, ay, alpha, Translation2d.kZero);
  }

  /**
   * Calculates wheel force feedforwards for the desired robot acceleration. This can be used with
   * either robot-centric or field-centric accelerations; the returned force feedforwards will match
   * in being robot-/field-centric.
   *
   * @param ax Acceleration in the X direction (forward) in m/s^2
   * @param ay Acceleration in the Y direction (left) in m/s^2
   * @param alpha Angular acceleration in rad/s^2
   * @param centerOfRotation Center of rotation
   * @return Wheel force feedforwards to apply
   */
  public final Feedforwards calculate(
      double ax, double ay, double alpha, Translation2d centerOfRotation) {
    final double fx = ax * mass;
    final double fy = ay * mass;
    final double tau = alpha * moi;

    final var feedforwards = new Feedforwards(moduleLocations.length);
    for (int i = 0; i < moduleLocations.length; ++i) {
      final var r = moduleLocations[i].minus(centerOfRotation);

      final var f_tau =
          new Translation2d(tau / r.getNorm(), r.getAngle().plus(Rotation2d.kCCW_90deg));
      feedforwards.x_newtons[i] = (fx + f_tau.getX()) / moduleLocations.length;
      feedforwards.y_newtons[i] = (fy + f_tau.getY()) / moduleLocations.length;
    }

    return feedforwards;
  }

  /**
   * Calculates wheel force feedforwards for the desired robot acceleration. This can be used with
   * either robot-centric or field-centric accelerations; the returned force feedforwards will match
   * in being robot-/field-centric.
   *
   * @param ax Acceleration in the X direction (forward) in m/s^2
   * @param ay Acceleration in the Y direction (left) in m/s^2
   * @param alpha Angular acceleration in rad/s^2
   * @return Wheel force feedforwards to apply
   */
  public final Feedforwards calculate(
      LinearAcceleration ax, LinearAcceleration ay, AngularAcceleration alpha) {
    return calculate(
        ax.in(MetersPerSecondPerSecond),
        ay.in(MetersPerSecondPerSecond),
        alpha.in(RadiansPerSecondPerSecond));
  }

  /**
   * Calculates wheel force feedforwards for the desired robot acceleration. This can be used with
   * either robot-centric or field-centric accelerations; the returned force feedforwards will match
   * in being robot-/field-centric.
   *
   * @param ax Acceleration in the X direction (forward) in m/s^2
   * @param ay Acceleration in the Y direction (left) in m/s^2
   * @param alpha Angular acceleration in rad/s^2
   * @param centerOfRotation Center of rotation
   * @return Wheel force feedforwards to apply
   */
  public final Feedforwards calculate(
      LinearAcceleration ax,
      LinearAcceleration ay,
      AngularAcceleration alpha,
      Translation2d centerOfRotation) {
    return calculate(
        ax.in(MetersPerSecondPerSecond),
        ay.in(MetersPerSecondPerSecond),
        alpha.in(RadiansPerSecondPerSecond),
        centerOfRotation);
  }

  /**
   * Calculates wheel force feedforwards for the desired change in speeds. This can be used with
   * either robot-centric or field-centric speeds; the returned force feedforwards will match in
   * being robot-/field-centric.
   *
   * @param dt The change in time between the path setpoints
   * @param prev The previous ChassisSpeeds setpoint of the path
   * @param current The new ChassisSpeeds setpoint of the path
   * @return Wheel force feedforwards to apply
   */
  public final Feedforwards calculate(double dt, ChassisSpeeds prev, ChassisSpeeds current) {
    return calculate(dt, prev, current, Translation2d.kZero);
  }

  /**
   * Calculates wheel force feedforwards for the desired change in speeds. This can be used with
   * either robot-centric or field-centric speeds; the returned force feedforwards will match in
   * being robot-/field-centric.
   *
   * @param dt The change in time between the path setpoints
   * @param prev The previous ChassisSpeeds setpoint of the path
   * @param current The new ChassisSpeeds setpoint of the path
   * @param centerOfRotation Center of rotation
   * @return Wheel force feedforwards to apply
   */
  public final Feedforwards calculate(
      double dt, ChassisSpeeds prev, ChassisSpeeds current, Translation2d centerOfRotation) {
    final double ax = (current.vxMetersPerSecond - prev.vxMetersPerSecond) / dt;
    final double ay = (current.vyMetersPerSecond - prev.vyMetersPerSecond) / dt;
    final double alpha = (current.omegaRadiansPerSecond - prev.omegaRadiansPerSecond) / dt;
    return calculate(ax, ay, alpha, centerOfRotation);
  }
}
