// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.PPUtil;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;

  public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(1.5);
  public static final AngularVelocity MaxModuleRate = RotationsPerSecond.of(20.0);

  // PathPlanner config constants
  private static final Mass ROBOT_MASS = Kilogram.of(32.36);
  private static final MomentOfInertia ROBOT_MOI =
      KilogramSquareMeters.of(ROBOT_MASS.magnitude() * (0.52705 / 2) * (0.0010165 / 0.00094588));
  private static final double WHEEL_COF = 1.5;
  public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = TunerConstants.FrontLeft;
  public static final Translation2d[] SWERVE_MODULE_OFFSETS =
      new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      };

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontRight.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS,
          ROBOT_MOI,
          new ModuleConfig(
              SWERVE_MODULE_CONSTANTS.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60(1).withReduction(SWERVE_MODULE_CONSTANTS.DriveMotorGearRatio),
              SWERVE_MODULE_CONSTANTS.SlipCurrent,
              1),
          SWERVE_MODULE_OFFSETS);

  public static final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(Constants.PP_CONFIG, Units.rotationsToRadians(10.0));

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Vision constants

  public static final String kFrontLeftCameraName = "Front Left";
  public static final String kFrontRightCameraName = "Front Right";
  public static final String kBackLeftCameraName = "Back Left";
  public static final String kbackRightCameraName = "Back Right";

  public static final Transform3d kFrontLeftStdDev = new Transform3d(
    new Translation3d(
      Units.inchesToMeters(9.91887103),
      Units.inchesToMeters(12.04442909),
      Units.inchesToMeters(8.55647482)),
    new Rotation3d(0, Units.degreesToRadians(25.16683805), Units.degreesToRadians(30)));

    public static final Transform3d kFrontRightStdDev = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(9.91887103),
        Units.inchesToMeters(-12.04442909),
        Units.inchesToMeters(8.55647482)),
      new Rotation3d(0, Units.degreesToRadians(25.16683805), Units.degreesToRadians(330)));
      
      public static final Transform3d kBackLeftStdDev = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(-9.79622433),
        Units.inchesToMeters(10.87979715),
        Units.inchesToMeters(8.55647482)),
      new Rotation3d(0, Units.degreesToRadians(25.16683805), Units.degreesToRadians(210)));

      public static final Transform3d kBackRightStdDev = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(-9.79622433),
        Units.inchesToMeters(-10.87979715),
        Units.inchesToMeters(8.55647482)),
      new Rotation3d(0, Units.degreesToRadians(25.16683805), Units.degreesToRadians(150)));

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  static {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      // Check if the GUI settings match the constants
      PPUtil.compareConfigs(config, PP_CONFIG);
    } catch (Exception e) {
      PPUtil.badGUI();
      e.printStackTrace();
    }
  }
}
