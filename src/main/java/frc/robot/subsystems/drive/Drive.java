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
import com.pathplanner.lib.util.PathPlannerLogging;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.requests.SysIdSwerveTranslation_Torque;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;
import frc.robot.utils.ArrayBuilder;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TargetingComputer.Targets;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Drive extends SubsystemBase {

  // Load the path we want to pathfind to and follow
  // private PathPlannerPath path = PathPlannerPath.fromPathFile("Align Alpha");

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs;
  private final ModuleIOInputsAutoLogged[] modules = ArrayBuilder.buildModuleAutoLogged();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.SWERVE_MODULE_OFFSETS);
  private SwerveDrivePoseEstimator poseEstimator = null;
  private Trigger estimatorTrigger =
      new Trigger(() -> poseEstimator != null).and(() -> Constants.currentMode == Mode.REPLAY);
  private SwerveModulePosition[] currentPositions = ArrayBuilder.buildSwerveModulePosition();

  private Alert[] driveDisconnectedAlert =
      ArrayBuilder.buildAlert("Disconnected drive motor on module");
  private Alert[] turnDisconnectedAlert =
      ArrayBuilder.buildAlert("Disconnected turn motor on module");
  private Alert[] turnEncoderDisconnectedAlert =
      ArrayBuilder.buildAlert("Disconnected turn encoder on module");

  private Alert gyroDisconnectedAlert = new Alert("Gyro Disconnected", AlertType.kError);

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve request to apply when braking */
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  // Example TorqueCurrent SysID - Others are avalible.
  private final SysIdSwerveTranslation_Torque m_translationTorqueCharacterization =
      new SysIdSwerveTranslation_Torque();

  /* SysId routine for characterizing torque translation. This is used to find PID gains for Torque Current of the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTorqueTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // Use ramp rate of 5 A/s
              Volts.of(10), // Use dynamic step of 10 A
              Seconds.of(5), // Use timeout of 5 seconds
              // Log state with SignalLogger class
              state -> Logger.recordOutput("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output ->
                  setControl(
                      m_translationTorqueCharacterization.withTorqueCurrent(
                          output.in(Volts))), // treat volts as amps
              null,
              this));

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Logger.recordOutput("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Logger.recordOutput("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second squared, but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Logger.recordOutput("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                Logger.recordOutput("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  public Drive(DriveIO io) {

    this.io = io;
    inputs = new DriveIOInputsAutoLogged();

    configureAutoBuilder();

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
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
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this // Subsystem for requirements
        );
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> io.setControl(requestSupplier.get()));
  }

  public BooleanSupplier hasSpeed(Targets target, Vision vision) {
    BooleanSupplier yes =
        (() ->
            TargetingComputer.getSelectTargetBranchPose(target).getX() - getPose().getX() < 0.05);
    return yes;
  }

  public Command Alignment(
      RobotCentric requestSupplier, Targets target, Vision vision, Elevator elevator) {
    if (elevator.isAtTarget()) {
      return run(() ->
              io.setControl(
                  requestSupplier
                      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                      .withVelocityX(
                          TunerConstants.kSpeedAt12Volts.times(
                              .75
                                  * (TargetingComputer.getSelectTargetBranchPose(target).getX()
                                      - getPose().getX())
                                  * 0.5))
                      .withVelocityY(
                          TunerConstants.kSpeedAt12Volts.times(
                              .75
                                  * (TargetingComputer.getSelectTargetBranchPose(target).getY()
                                      - getPose().getY())
                                  * 0.5))
                      .withRotationalRate(
                          Constants.MaxAngularRate.times(
                              (new Rotation2d(Units.degreesToRadians(target.getTargetingAngle()))
                                      .minus(getPose().getRotation())
                                      .getRadians())
                                  * 0.4))))
          // .withVelocityX(
          //     TunerConstants.kSpeedAt12Volts.times(
          //         -(target.getOffset().getX()
          //                 - vision
          //                     .calculateOffset(target.getApriltag(), target.getOffset())
          //                     .getX())
          //             * 0.25))
          // .withVelocityY(
          //     TunerConstants.kSpeedAt12Volts.times(
          //         -(target.getOffset().getY()
          //                 - vision
          //                     .calculateOffset(target.getApriltag(), target.getOffset())
          //                     .getY())
          //             * 0.25))
          // .withRotationalRate(
          //     Constants.MaxAngularRate.times(
          //         (new Rotation2d(Units.degreesToRadians(target.getTargetingAngle()))
          //                 .minus(getPose().getRotation())
          //                 .getRadians())
          //             * 0.25))))
          .until(hasSpeed(target, vision))
          .finallyDo(
              () ->
                  io.setControl(
                      requestSupplier
                          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                          .withVelocityX(0)
                          .withVelocityY(0)
                          .withRotationalRate(0)));
    } else {
      return run(() -> elevator.setElevatorPosition(elevator.getMode()));
    }
  }

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
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */

    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    gyroDisconnectedAlert.set(!inputs.gyroConnected);

    io.updateModules(modules);
    for (int i = 0; i < modules.length; i++) {
      Logger.processInputs("Module" + i, modules[i]);
      driveDisconnectedAlert[i].set(!modules[i].driveConnected);
      turnDisconnectedAlert[i].set(!modules[i].turnConnected);
      turnEncoderDisconnectedAlert[i].set(!modules[i].turnEncoderConnected);
    }

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

    Logger.recordOutput(
        "distance to target",
        Units.metersToInches(
            getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()));
  }

  public void resetPose(Pose2d pose) {
    if (estimatorTrigger.getAsBoolean()) {
      poseEstimator.resetPose(pose);
    }
    io.resetPose(pose);
  }

  /*
  public Command goToPoint(int x, int y) {
    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));
    PathConstraints constraints =
        new PathConstraints(4.0, 5.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    return AutoBuilder.pathfindToPose(targetPose, constraints);
  }
  /*
   * flips if needed
   */
  /*
  public Command goToPoint(Pose2d pose) {
    PathConstraints constraints =
        new PathConstraints(3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    return new ConditionalCommand(
        AutoBuilder.pathfindToPoseFlipped(pose, constraints),
        AutoBuilder.pathfindToPose(pose, constraints),
        () -> Robot.getAlliance());
  }*/

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    if (estimatorTrigger.getAsBoolean()) {
      return poseEstimator.getEstimatedPosition();
    }
    return inputs.pose;
  }

  public Translation2d getDistanceToPose(Pose2d pose) {
    Pose2d currentPose = getPose();
    return pose.minus(currentPose).getTranslation().unaryMinus();
  }

  @AutoLogOutput
  public boolean isInAlignmentZone() {
    Pose2d currentPose = getPose();
    double zoneRadius = TargetingComputer.alignmentRange;
    double angle =
        Robot.getAlliance()
            ? new Translation2d(
                    FieldConstants.fieldLength.magnitude(), FieldConstants.fieldWidth.magnitude())
                .minus(FieldConstants.Reef.center)
                .minus(currentPose.getTranslation())
                .unaryMinus()
                .getAngle()
                .getDegrees()
            : FieldConstants.Reef.center
                .minus(currentPose.getTranslation())
                .unaryMinus()
                .getAngle()
                .getDegrees();
    int zone;

    if (angle >= -30 && angle < 30) { // Golf / Alpha
      zone = 4;
    } else if (angle >= 30 && angle < 90) { // India / Charlie
      zone = 5;
    } else if (angle >= 90 && angle < 150) { // Kilo / Echo
      zone = 6;
    } else if (angle >= -150 && angle < -90) { // Charlie / India
      zone = 2;
    } else if (angle >= -90 && angle < -30) { // Echo / Kilo
      zone = 3;
    } else { // Alpha / Golf
      zone = 1;
    }

    switch (TargetingComputer.getCurrentTargetBranch().getApriltag()) {
      default:
        return false;
      case (6):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 3;
      case (7):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 4;
      case (8):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 5;
      case (9):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 6;
      case (10):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 1;
      case (11):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 2;
      case (17):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 2;
      case (18):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 1;
      case (19):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 6;
      case (20):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 5;
      case (21):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 4;
      case (22):
        return getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
                < zoneRadius
            && zone == 3;
    }
  }

  public BooleanSupplier inAlignmentZone(Targets target) {
    TargetingComputer.setTargetBranch(target);
    return () -> isInAlignmentZone();
  }

  @AutoLogOutput
  public boolean isNearProcessor() {
    new Translation2d(FieldConstants.fieldLength.magnitude(), FieldConstants.fieldWidth.magnitude())
        .minus(FieldConstants.Processor.centerFace.getTranslation());
    Pose2d processor =
        Robot.getAlliance()
            ? new Pose2d(
                new Translation2d(
                        FieldConstants.fieldLength.magnitude(),
                        FieldConstants.fieldWidth.magnitude())
                    .minus(FieldConstants.Processor.centerFace.getTranslation()),
                FieldConstants.Processor.centerFace.getRotation().minus(new Rotation2d(Math.PI)))
            : FieldConstants.Processor.centerFace;

    return getDistanceToPose(processor).getNorm() < 1.25;
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public AngularVelocity getGyroRate() {
    return inputs.gyroRate;
  }

  public Rotation2d getOperatorForwardDirection() {
    return inputs.operatorForwardDirection;
  }

  public Angle[] getDrivePositions() {
    Angle[] values = new Angle[Constants.PP_CONFIG.numModules];
    for (int i = 0; i < values.length; i++) {
      values[i] = modules[i].drivePosition;
    }
    return values;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    return inputs.moduleStates;
  }

  /** Returns the module target states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Setpoints")
  public SwerveModuleState[] getModuleTarget() {
    return inputs.moduleTargets;
  }

  public SwerveModulePosition[] getModulePositions() {
    return inputs.modulePositions;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return inputs.speeds;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured/velocity")
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
    this.addVisionMeasurement(
        new Pose2d(
            new Translation2d(
                visionMeasurement.poseEstimate().pose().toPose2d().getX(),
                visionMeasurement.poseEstimate().pose().toPose2d().getY()),
            this.getRotation()),
        visionMeasurement.poseEstimate().timestampSeconds(),
        visionMeasurement.visionMeasurementStdDevs());
  }

  public void addVisionData(List<VisionMeasurement> visionData) {
    visionData.forEach(this::addVisionMeasurement);
  }

  public VisionParameters getVisionParameters() {
    return new VisionParameters(getPose(), getGyroRate());
  }

  public record VisionParameters(Pose2d robotPose, AngularVelocity gyroRate) {}

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
