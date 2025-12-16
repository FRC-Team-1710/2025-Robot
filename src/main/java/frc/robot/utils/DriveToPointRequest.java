package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.WheelForceCalculator.Feedforwards;

public class DriveToPointRequest implements SwerveRequest {
  /** Meters */
  private double maxErrorTranslation = 0;

  /** Degrees */
  private double maxErrorTheta = 0;

  private double frictionCoefficient = 0;

  private boolean atTarget = false;

  public Pose2d TargetPose = Pose2d.kZero;

  // PID Controllers
  public PhoenixPIDController XController = new PhoenixPIDController(10, 0, 0);
  public PhoenixPIDController YController = new PhoenixPIDController(10, 0, 0);
  public PhoenixPIDController ThetaController = new PhoenixPIDController(7, 0, 0);

  private final WheelForceCalculator forceCalculator;
  private final LinearPath path;

  // State
  private double elapsedTime = 0;
  private LinearPath.State initialState = new LinearPath.State();
  private ChassisSpeeds lastSetpointSpeeds = new ChassisSpeeds();
  private LinearPath.State setpoint = new LinearPath.State();

  // Underlying request
  private final SwerveRequest.ApplyFieldSpeeds driveRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  public DriveToPointRequest(
      TrapezoidProfile.Constraints linearProfile,
      TrapezoidProfile.Constraints angularProfile,
      WheelForceCalculator forceCalculator) {

    this.forceCalculator = forceCalculator;
    this.path = new LinearPath(linearProfile, angularProfile);
    ThetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    initialState =
        new LinearPath.State(
            currentPose,
            ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation()));

    elapsedTime = 0.0;
    // Initialize setpoint to current state
    setpoint = path.calculate(elapsedTime, initialState, TargetPose);
    lastSetpointSpeeds = setpoint.speeds;
  }

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    double dt = parameters.updatePeriod;
    elapsedTime += dt;

    // Update setpoint to get speed robot should be running at based on current time
    setpoint = path.calculate(elapsedTime, initialState, TargetPose);

    Pose2d currentPose = parameters.currentPose;

    // Calculate feedback corrections
    double xFeedback =
        XController.calculate(currentPose.getX(), setpoint.pose.getX(), parameters.timestamp);
    double yFeedback =
        YController.calculate(currentPose.getY(), setpoint.pose.getY(), parameters.timestamp);
    double thetaFeedback =
        ThetaController.calculate(
            currentPose.getRotation().getRadians(),
            setpoint.pose.getRotation().getRadians(),
            parameters.timestamp);

    ChassisSpeeds feedbackSpeeds = new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback);

    // Calculate feedforward forces based on PLANNED trajectory change
    Feedforwards feedforwards = forceCalculator.calculate(dt, lastSetpointSpeeds, setpoint.speeds);

    boolean transAtTarget =
        Math.abs(TargetPose.getTranslation().getDistance(currentPose.getTranslation()))
            <= maxErrorTranslation;
    boolean rotAtTarget =
        Math.abs(
                TargetPose.getRotation()
                    .getMeasure()
                    .minus(currentPose.getRotation().getMeasure())
                    .in(Degrees))
            <= maxErrorTheta;

    atTarget = transAtTarget && rotAtTarget;

    // If at target stop moving
    ChassisSpeeds correctedSpeeds = new ChassisSpeeds();

    if (!atTarget) {
      // Combine feedforward + feedback
      correctedSpeeds =
          new ChassisSpeeds(
              setpoint.speeds.vxMetersPerSecond
                  + feedbackSpeeds.vxMetersPerSecond
                  + frictionCoefficient,
              setpoint.speeds.vyMetersPerSecond
                  + feedbackSpeeds.vyMetersPerSecond
                  + frictionCoefficient,
              setpoint.speeds.omegaRadiansPerSecond + feedbackSpeeds.omegaRadiansPerSecond);
    }

    // Save setpoint speeds for next feedforward calculation
    lastSetpointSpeeds = setpoint.speeds;

    // Apply feedforward forces (from planned trajectory) + corrected speeds
    // (planned + feedback)
    return driveRequest
        .withSpeeds(correctedSpeeds)
        .withWheelForceFeedforwardsX(feedforwards.x_newtons)
        .withWheelForceFeedforwardsY(feedforwards.y_newtons)
        .apply(parameters, modulesToApply);
  }

  /**
   * @return True if the profiled path is finished, not if the robot is {@link #atTarget()}
   */
  public boolean isFinished() {
    return path.isFinished(elapsedTime);
  }

  public boolean atTarget() {
    return atTarget;
  }

  public DriveToPointRequest withFrictionCoefficient(LinearVelocity frictionCoefficient) {
    this.frictionCoefficient = frictionCoefficient.in(MetersPerSecond);
    return this;
  }

  public DriveToPointRequest withFrictionCoefficient(double frictionCoefficient) {
    this.frictionCoefficient = frictionCoefficient;
    return this;
  }

  public DriveToPointRequest withMaxErrorTranslation(Distance maxErrorTranslation) {
    this.maxErrorTranslation = maxErrorTranslation.in(Meters);
    return this;
  }

  public DriveToPointRequest withMaxErrorTranslation(double maxErrorTranslation) {
    this.maxErrorTranslation = maxErrorTranslation;
    return this;
  }

  public DriveToPointRequest withMaxErrorTheta(Angle maxErrorTheta) {
    this.maxErrorTheta = maxErrorTheta.in(Degrees);
    return this;
  }

  public DriveToPointRequest withMaxErrorTheta(double maxErrorTheta) {
    this.maxErrorTheta = maxErrorTheta;
    return this;
  }

  public DriveToPointRequest withTargetPose(Pose2d targetPose) {
    this.TargetPose = targetPose;
    return this;
  }

  public DriveToPointRequest withXController(PhoenixPIDController controller) {
    this.XController = controller;
    return this;
  }

  public DriveToPointRequest withYController(PhoenixPIDController controller) {
    this.YController = controller;
    return this;
  }

  public DriveToPointRequest withThetaController(PhoenixPIDController controller) {
    this.ThetaController = controller;
    return this;
  }
}
