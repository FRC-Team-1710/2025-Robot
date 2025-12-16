// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.utils.ArrayBuilder;
import java.util.Optional;

/**
 * Interface for drive subsystem I/O operations. Handles swerve drive state management and pose
 * estimation.
 */
@Logged
public interface DriveIO {
  @Logged
  public static class DriveIOInputs {
    // Module arrays with default states
    @Logged(name = "ModuleStates", importance = Importance.DEBUG)
    public SwerveModuleState[] moduleStates = ArrayBuilder.buildSwerveModuleState();

    @Logged(name = "ModuleTargets", importance = Importance.CRITICAL)
    public SwerveModuleState[] moduleTargets = ArrayBuilder.buildSwerveModuleState();

    @Logged(name = "ModulePositions", importance = Importance.CRITICAL)
    public SwerveModulePosition[] modulePositions = ArrayBuilder.buildSwerveModulePosition();

    // Position and motion state
    @Logged(name = "Pose", importance = Importance.CRITICAL)
    public Pose2d pose = Pose2d.kZero;

    @Logged(name = "Speeds", importance = Importance.CRITICAL)
    public ChassisSpeeds speeds = new ChassisSpeeds();

    @Logged(name = "OperatorForwardDirection", importance = Importance.DEBUG)
    public Rotation2d operatorForwardDirection = new Rotation2d(Units.degreesToRadians(90));

    // Diagnostic data
    @Logged(name = "OdometryPeriod", importance = Importance.DEBUG)
    public double odometryPeriod = 0.0;

    @Logged(name = "SuccessfulDaqs", importance = Importance.DEBUG)
    public int successfulDaqs = 0;

    @Logged(name = "FailedDaqs", importance = Importance.DEBUG)
    public int failedDaqs = 0;

    @Logged(name = "OdometryIsValid", importance = Importance.DEBUG)
    public boolean odometryIsValid = false;

    @Logged(name = "Roll", importance = Importance.INFO)
    public Angle roll = Degrees.of(0);

    @Logged(name = "Pitch", importance = Importance.INFO)
    public Angle pitch = Degrees.of(0);

    @Logged(name = "Yaw", importance = Importance.INFO)
    public Angle yaw = Degrees.of(0);

    @Logged(name = "Rotation3d", importance = Importance.DEBUG)
    public Rotation3d rotation3d = new Rotation3d();

    @Logged(name = "SupplyVoltage", importance = Importance.DEBUG)
    public Voltage supplyVoltage = Volts.of(0);

    @Logged(name = "TemperatureC", importance = Importance.DEBUG)
    public double temperatureC = 0;

    @Logged(name = "AccumZ", importance = Importance.DEBUG)
    public Angle accumZ = Degrees.of(0);

    @Logged(name = "AccumY", importance = Importance.DEBUG)
    public Angle accumY = Degrees.of(0);

    @Logged(name = "AccumX", importance = Importance.DEBUG)
    public Angle accumX = Degrees.of(0);

    @Logged(name = "AcelZ", importance = Importance.DEBUG)
    public LinearAcceleration acelz = MetersPerSecondPerSecond.of(0);

    @Logged(name = "AcelY", importance = Importance.DEBUG)
    public LinearAcceleration acely = MetersPerSecondPerSecond.of(0);

    @Logged(name = "AcelX", importance = Importance.DEBUG)
    public LinearAcceleration acelx = MetersPerSecondPerSecond.of(0);

    // Sensor data
    @Logged(name = "Timestamp", importance = Importance.INFO)
    public double[] timestamp = new double[0];

    @Logged(name = "GyroYaw", importance = Importance.INFO)
    public Rotation2d[] gyroYaw = new Rotation2d[0];

    @Logged(name = "GyroRate", importance = Importance.CRITICAL)
    public AngularVelocity gyroRate = RotationsPerSecond.of(0.0);

    @Logged(name = "GyroConnected", importance = Importance.CRITICAL)
    public boolean gyroConnected = false;

    // Module position arrays
    @Logged(name = "DrivePositions", importance = Importance.DEBUG)
    public double[][] drivePositions = new double[Constants.PP_CONFIG.numModules][0];

    @Logged(name = "SteerPositions", importance = Importance.DEBUG)
    public Rotation2d[][] steerPositions = new Rotation2d[Constants.PP_CONFIG.numModules][0];
  }

  @Logged
  public static class ModuleIOInputs {
    @Logged(name = "DriveConnected", importance = Importance.CRITICAL)
    public boolean driveConnected = false;

    @Logged(name = "DrivePosition", importance = Importance.CRITICAL)
    public Angle drivePosition = Radians.of(0.0);

    @Logged(name = "DriveVelocity", importance = Importance.CRITICAL)
    public AngularVelocity driveVelocity = RotationsPerSecond.of(0.0);

    @Logged(name = "DriveAppliedVolts", importance = Importance.CRITICAL)
    public Voltage driveAppliedVolts = Volt.of(0.0);

    @Logged(name = "DriveStatorCurrent", importance = Importance.CRITICAL)
    public Current driveStatorCurrent = Amps.of(0.0);

    @Logged(name = "DriveSupplyCurrent", importance = Importance.INFO)
    public Current driveSupplyCurrent = Amps.of(0.0);

    @Logged(name = "TurnConnected", importance = Importance.CRITICAL)
    public boolean turnConnected = false;

    @Logged(name = "TurnEncoderConnected", importance = Importance.CRITICAL)
    public boolean turnEncoderConnected = false;

    @Logged(name = "TurnAbsolutePosition", importance = Importance.CRITICAL)
    public Angle turnAbsolutePosition = Rotations.of(0.0);

    @Logged(name = "TurnPosition", importance = Importance.CRITICAL)
    public Angle turnPosition = Rotations.of(0.0);

    @Logged(name = "TurnVelocity", importance = Importance.CRITICAL)
    public AngularVelocity turnVelocity = RotationsPerSecond.of(0.0);

    @Logged(name = "TurnAppliedVolts", importance = Importance.CRITICAL)
    public Voltage turnAppliedVolts = Volt.of(0.0);

    @Logged(name = "TurnStatorCurrent", importance = Importance.CRITICAL)
    public Current turnStatorCurrent = Amps.of(0.0);

    @Logged(name = "TurnSupplyCurrent", importance = Importance.INFO)
    public Current turnSupplyCurrent = Amps.of(0.0);
  }

  default void updateInputs(DriveIOInputs inputs) {}

  default void updateModules(ModuleIOInputs[] inputs) {}

  default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  default void setControl(SwerveRequest request) {}

  default void resetPose(Pose2d pose) {}

  @NotLogged
  default Optional<Pose2d> samplePoseAt(double timestamp) {
    return Optional.empty();
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * PoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link #updateWithTime}, then you must use a
   *     timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
   *     epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that you
   *     should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source in
   *     this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}
}
