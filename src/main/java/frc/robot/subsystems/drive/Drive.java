// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.autos.SysIdSwerveTranslationTorqueCurrentFOC;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;
import frc.robot.subsystems.drive.DriveIO.ModuleIOInputs;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;
import frc.robot.utils.ArrayBuilder;
import java.util.List;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveClass and implements Subsystem so it can easily be used in
 * command-based projects.
 */
@Logged
public class Drive extends SubsystemBase {

  // Load the path we want to pathfind to and follow
  // private PathPlannerPath path = PathPlannerPath.fromPathFile("Align Alpha");

  @Logged(name = "IO", importance = Importance.CRITICAL)
  private final DriveIO io;

  @Logged(name = "Inputs", importance = Importance.CRITICAL)
  private final DriveIOInputs inputs;

  @Logged(name = "Modules", importance = Importance.CRITICAL)
  private final ModuleIOInputs[] modules = ArrayBuilder.buildModuleInputs();

  @NotLogged
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.SWERVE_MODULE_OFFSETS);

  @NotLogged private SwerveDrivePoseEstimator poseEstimator = null;

  @Logged(name = "RecentVisionMeasurement", importance = Importance.INFO)
  private VisionMeasurement recentVisionMeasurement = null;

  @Logged(name = "EstimatorTrigger", importance = Importance.DEBUG)
  private Trigger estimatorTrigger =
      new Trigger(() -> poseEstimator != null).and(() -> Constants.currentMode == Mode.REPLAY);

  @Logged(name = "CurrentPositions", importance = Importance.CRITICAL)
  private SwerveModulePosition[] currentPositions = ArrayBuilder.buildSwerveModulePosition();

  // private Alert[] driveDisconnectedAlert =
  //     ArrayBuilder.buildAlert("Disconnected drive motor on module");
  // private Alert[] turnDisconnectedAlert =
  //     ArrayBuilder.buildAlert("Disconnected turn motor on module");
  // private Alert[] turnEncoderDisconnectedAlert =
  //     ArrayBuilder.buildAlert("Disconnected turn encoder on module");

  // private Alert gyroDisconnectedAlert = new Alert("Gyro Disconnected", AlertType.kError);

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  @Logged(name = "BlueAlliancePerspectiveRotation", importance = Importance.DEBUG)
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  @Logged(name = "RedAlliancePerspectiveRotation", importance = Importance.DEBUG)
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  /* Keep track if we've ever applied the operator perspective before or not */
  @Logged(name = "HasAppliedOperatorPerspective", importance = Importance.INFO)
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  @NotLogged
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve request to apply when braking */
  @NotLogged
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  @NotLogged
  private final SysIdSwerveTranslationTorqueCurrentFOC m_rotationCharacterization =
      new SysIdSwerveTranslationTorqueCurrentFOC();

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  // private final SysIdRoutine m_sysIdRoutineRotation =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             /*
  //              * This is in radians per second squared, but SysId only supports
  //              * "volts per second"
  //              */
  //             Volts.of(Math.PI / 6).per(Second),
  //             /* This is in radians per second, but SysId only supports "volts" */
  //             Volts.of(Math.PI),
  //             null, // Use default timeout (10 s)
  //             // Log state with Logger class
  //             state -> Logger.recordOutput("SysIdRotation_State", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             output -> {
  //               /* output is actually radians per second, but SysId only supports "volts" */
  //               setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
  //               /* also log the requested output for SysId */
  //               Logger.recordOutput("Rotational_Rate", output.in(Volts));
  //             },
  //             null,
  //             this));

  @NotLogged private final SysIdRoutine m_sysIdRoutineRotation = null;
  // new SysIdRoutine(
  //     new SysIdRoutine.Config(
  //         /*
  //          * This is in radians per second squared, but SysId only supports
  //          * "volts per second"
  //          */
  //         Volts.of(6).per(Second),
  //         /* This is in radians per second, but SysId only supports "volts" */
  //         Volts.of(10),
  //         null, // Use default timeout (10 s)
  //         // Log state with Logger class
  //         state -> Epi,
  //     new SysIdRoutine.Mechanism(
  //         output -> {
  //           /* output is actually radians per second, but SysId only supports "volts" */
  //           setControl(m_rotationCharacterization.withCurrent(output.in(Volts)));
  //           /* also log the requested output for SysId */
  //           Logger.recordOutput("Translation_Rate", output.in(Volts));
  //         },
  //         null,
  //         this));

  /* The SysId routine to test */
  @NotLogged private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

  public Drive(DriveIO io) {

    this.io = io;
    inputs = new DriveIOInputs();

    configureAutoBuilder();

    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });
  }

  private void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose, // Supplier of current robot pose
        this::resetPose, // Consumer for seeding pose against auto
        this::getChassisSpeeds, // Supplier of current robot speeds
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        (speeds, feedforwards) ->
            io.setControl(
                m_pathApplyRobotSpeeds
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        new PPHolonomicDriveController(
            // PID constants for translation
            new PIDConstants(10, 0, 0),
            // PID constants for rotation
            new PIDConstants(7, 0, 0)),
        Constants.PP_CONFIG,
        // Assume the path needs to be flipped for Red vs Blue, this is normally the
        // case
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this // Subsystem for requirements
        );
  }

  /**
   * Returns a command that applies the specified control request to this swerve
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  @NotLogged
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> io.setControl(requestSupplier.get()));
  }

  // Logged elsewhere
  @NotLogged
  public ChassisSpeeds getChassisSpeeds() {
    return inputs.speeds;
  }

  @NotLogged
  public Command stop(RobotCentric requestSupplier) {
    return run(
        () ->
            io.setControl(
                requestSupplier
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)));
  }

  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  @NotLogged
  public Command brake() {
    return applyRequest(() -> brakeRequest);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  @NotLogged
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  @NotLogged
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */

    io.updateInputs(inputs);

    io.updateModules(modules);

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                io.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
    updateWithTime();
  }

  public void resetPose(Pose2d pose) {
    if (estimatorTrigger.getAsBoolean()) {
      poseEstimator.resetPose(pose);
    }
    io.resetPose(pose);
  }

  /*
   * public Command goToPoint(int x, int y) {
   * Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));
   * PathConstraints constraints =
   * new PathConstraints(4.0, 5.0, Units.degreesToRadians(540),
   * Units.degreesToRadians(720));
   * return AutoBuilder.pathfindToPose(targetPose, constraints);
   * }
   * /*
   * flips if needed
   */
  /*
   * public Command goToPoint(Pose2d pose) {
   * PathConstraints constraints =
   * new PathConstraints(3.0, 2.0, Units.degreesToRadians(540),
   * Units.degreesToRadians(720));
   * return new ConditionalCommand(
   * AutoBuilder.pathfindToPoseFlipped(pose, constraints),
   * AutoBuilder.pathfindToPose(pose, constraints),
   * () -> Robot.getAlliance());
   * }
   */

  /** Returns the current odometry pose. */
  // Logged elsewhere
  @NotLogged
  public Pose2d getPose() {
    // return new Pose2d(new Translation2d(8, 6), inputs.pose.getRotation());
    if (estimatorTrigger.getAsBoolean()) {
      return poseEstimator.getEstimatedPosition();
    }
    return inputs.pose;
  }

  public Translation2d getDistanceToPose(Pose2d pose) {
    Pose2d currentPose = getPose();
    return pose.minus(currentPose).getTranslation().unaryMinus();
  }

  @NotLogged
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @NotLogged
  public AngularVelocity getGyroRate() {
    return inputs.gyroRate;
  }

  @NotLogged
  public Rotation2d getOperatorForwardDirection() {
    return inputs.operatorForwardDirection;
  }

  @NotLogged
  public Angle[] getDrivePositions() {
    Angle[] values = new Angle[Constants.PP_CONFIG.numModules];
    for (int i = 0; i < values.length; i++) {
      values[i] = modules[i].drivePosition;
    }
    return values;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @NotLogged
  public SwerveModuleState[] getModuleStates() {
    return inputs.moduleStates;
  }

  /** Returns the module target states (turn angles and drive velocities) for all of the modules. */
  @NotLogged
  public SwerveModuleState[] getModuleTarget() {
    return inputs.moduleTargets;
  }

  @NotLogged
  public SwerveModulePosition[] getModulePositions() {
    return inputs.modulePositions;
  }

  /** Returns the measured chassis speeds of the robot. */
  @Logged(name = "Velocity(m/s)", importance = Importance.CRITICAL)
  public double getChassisVelocity() {

    return Math.sqrt(
        Math.pow(inputs.speeds.vxMetersPerSecond, 2)
            + Math.pow(inputs.speeds.vyMetersPerSecond, 2));
  }

  /**
   * Return the pose at a given timestamp. If the buffer is empty return current pose.
   *
   * @param timestampSeconds The pose's timestamp. This must use WPILib timestamp.
   * @return The pose at the given timestamp (or current pose if the buffer is empty).
   */
  @NotLogged
  public Pose2d samplePoseAt(double timestampSeconds) {
    return estimatorTrigger.getAsBoolean()
        ? poseEstimator.sampleAt(timestampSeconds).orElse(getPose())
        : io.samplePoseAt(timestampSeconds).orElse(getPose());
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionRobotPoseMeters The measured robot pose from vision
   * @param timestampSeconds The timestamp of the measurement
   * @param visionMeasurementStdDevs Standard deviation matrix for the measurement
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (estimatorTrigger.getAsBoolean()) {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    } else {
      io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(VisionMeasurement visionMeasurement) {
    // Logger.recordOutput(
    //     "Odom minus Vision",
    //     this.getRotation().getRadians()
    //         - visionMeasurement.poseEstimate().pose().getRotation().getZ());
    Pose2d poseEstimate =
        new Pose2d(
            new Translation2d(
                visionMeasurement.poseEstimate().pose().toPose2d().getX(),
                visionMeasurement.poseEstimate().pose().toPose2d().getY()),
            visionMeasurement.poseEstimate().pose().toPose2d().getRotation());
    this.addVisionMeasurement(
        poseEstimate,
        visionMeasurement.poseEstimate().timestampSeconds(),
        visionMeasurement.visionMeasurementStdDevs());

    // Add the most accurate and recent vision measurement
    if (this.recentVisionMeasurement == null) {
      this.recentVisionMeasurement = visionMeasurement;
    }

    // Just in case this doesn't work at all (which it should)
    try {
      double timestamp = this.recentVisionMeasurement.poseEstimate().timestampSeconds();
      if (visionMeasurement.poseEstimate().timestampSeconds() < timestamp + 0.01) {
        if (visionMeasurement.poseEstimate().ambiguity()
            < this.recentVisionMeasurement.poseEstimate().ambiguity()) {
          this.recentVisionMeasurement = visionMeasurement;
        }
      } else {
        this.recentVisionMeasurement = visionMeasurement;
      }
    } catch (Error e) {
    }
  }

  public void addVisionData(List<VisionMeasurement> visionData) {
    visionData.forEach(this::addVisionMeasurement);
  }

  @NotLogged
  public VisionParameters getVisionParameters() {
    return new VisionParameters(getPose(), getGyroRate());
  }

  public record VisionParameters(Pose2d robotPose, AngularVelocity gyroRate) {}

  public void poseWithVisionRotation() {
    // Just in case this doesn't work (it should)
    try {
      if (recentVisionMeasurement != null) {
        Rotation2d rotation =
            recentVisionMeasurement.poseEstimate().pose().getRotation().toRotation2d();
        poseEstimator.resetRotation(rotation);
        io.resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), rotation));
      }
    } catch (Error e) {
      if (estimatorTrigger.getAsBoolean()) {
        poseEstimator.resetPose(Pose2d.kZero);
      }
      io.resetPose(Pose2d.kZero);
    }
  }

  public void poseWithVisionTranslation() {
    // Just in case this doesn't work (it should)
    try {
      if (recentVisionMeasurement != null) {
        Translation2d translation =
            recentVisionMeasurement.poseEstimate().robotPose().getTranslation();
        poseEstimator.resetTranslation(translation);
        io.resetPose(new Pose2d(translation, poseEstimator.getEstimatedPosition().getRotation()));
      }
    } catch (Error e) {
      if (estimatorTrigger.getAsBoolean()) {
        poseEstimator.resetPose(Pose2d.kZero);
      }
      io.resetPose(Pose2d.kZero);
    }
  }

  public void poseWithVisionPose() {
    // Just in case this doesn't work (it should)
    try {
      if (recentVisionMeasurement != null) {
        Pose2d pose =
            new Pose2d(
                recentVisionMeasurement.poseEstimate().robotPose().getTranslation(),
                recentVisionMeasurement.poseEstimate().pose().getRotation().toRotation2d());
        if (estimatorTrigger.getAsBoolean()) {
          poseEstimator.resetPose(pose);
        }
        io.resetPose(pose);
      }
    } catch (Error e) {
      if (estimatorTrigger.getAsBoolean()) {
        poseEstimator.resetPose(Pose2d.kZero);
      }
      io.resetPose(Pose2d.kZero);
    }
  }

  public void updateWithTime() {
    if (Constants.currentMode != Mode.REPLAY || !inputs.odometryIsValid) {
      return;
    }

    if (!estimatorTrigger.getAsBoolean()) {
      poseEstimator =
          new SwerveDrivePoseEstimator(
              kinematics, inputs.pose.getRotation(), inputs.modulePositions, inputs.pose);
    }

    for (int timeIndex = 0; timeIndex < inputs.timestamp.length; timeIndex++) {
      updateModulePositions(timeIndex);
      poseEstimator.updateWithTime(
          inputs.timestamp[timeIndex], inputs.gyroYaw[timeIndex], currentPositions);
    }
  }

  private void updateModulePositions(int timeIndex) {
    for (int moduleIndex = 0; moduleIndex < currentPositions.length; moduleIndex++) {
      currentPositions[moduleIndex].distanceMeters = inputs.drivePositions[moduleIndex][timeIndex];
      currentPositions[moduleIndex].angle = inputs.steerPositions[moduleIndex][timeIndex];
    }
  }
}
