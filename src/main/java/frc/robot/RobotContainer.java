package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TargetingComputer.Targets;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController driver =
      new TunableController(0)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.125);

  private final TunableController mech =
      new TunableController(0)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.125);

  private final Joystick reefTargetingSystem = new Joystick(2);
  private final JoystickButton alphaButton = new JoystickButton(reefTargetingSystem, 1);
  private final JoystickButton bravoButton = new JoystickButton(reefTargetingSystem, 2);
  private final JoystickButton charlieButton = new JoystickButton(reefTargetingSystem, 3);
  private final JoystickButton deltaButton = new JoystickButton(reefTargetingSystem, 4);
  private final JoystickButton echoButton = new JoystickButton(reefTargetingSystem, 5);
  private final JoystickButton foxtrotButton = new JoystickButton(reefTargetingSystem, 6);
  private final JoystickButton golfButton = new JoystickButton(reefTargetingSystem, 7);
  private final JoystickButton hotelButton = new JoystickButton(reefTargetingSystem, 8);
  private final JoystickButton indiaButton = new JoystickButton(reefTargetingSystem, 9);
  private final JoystickButton julietButton = new JoystickButton(reefTargetingSystem, 10);
  private final JoystickButton kiloButton = new JoystickButton(reefTargetingSystem, 11);
  private final JoystickButton limaButton = new JoystickButton(reefTargetingSystem, 12);

  private final LoggedDashboardChooser<Command> autoChooser;

  VisionIOPhotonVision c1;

  public final Drive drivetrain;
  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.025))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.025)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentric =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed.times(0.025))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.025))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private Vision vision;

  public RobotContainer() {
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);

        /*
         * Vision Class for referencing.
         * Contains 4 cameras as well as their respective names and standard deviations from robot
         */
        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIOPhotonVision(
                    "FrontLeft",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(12.04442909),
                            Units.inchesToMeters(9.91887103),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(115.16683805),
                            Units.degreesToRadians(30)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "FrontRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(12.04442909),
                            Units.inchesToMeters(9.91887103),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(115.16683805),
                            Units.degreesToRadians(330)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackLeft",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.87979715),
                            -Units.inchesToMeters(9.79622433),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(115.16683805),
                            Units.degreesToRadians(150)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(10.87979715),
                            -Units.inchesToMeters(9.79622433),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(115.16683805),
                            Units.degreesToRadians(210)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);
        
        vision = new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVisionSIM(
                "Front Camera",
                new Transform3d(
                    new Translation3d(0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(0))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Back Camera",
                new Transform3d(
                    new Translation3d(-0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(180))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Left Camera",
                new Transform3d(
                    new Translation3d(0.0, 0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(90))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Right Camera",
                new Transform3d(
                    new Translation3d(0.0, -0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(-90))),
                drivetrain::getVisionParameters));
        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    /*c1 =
    new VisionIOPhotonVision(
        "FrontLeft",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.04442909),
                Units.inchesToMeters(9.91887103),
                Units.inchesToMeters(8.55647482)), // IN METERS
            new Rotation3d(
                0,
                Units.degreesToRadians(115.16683805),
                Units.degreesToRadians(30)) // IN RADIANS
            ),
        drivetrain::getVisionParameters);*/

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drivetrain));
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -driver.customLeft().getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -driver.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -driver
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X (left)

    driver.a().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

    // driver.back().whileTrue(drivetrain.goToPoint(3, 2));

    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
    /*driver
    .leftBumper()
    .onTrue(
        drivetrain.applyRequest(
            () -> point.withModuleDirection(new Rotation2d(c1.target(TargetingComputer.currentTargetBranch.getApriltag()).getBestCameraToTarget().getX()))));

            driver
            .leftBumper()
            .onTrue(
                drivetrain.applyRequest(
                    () -> point.withModuleDirection(new Rotation2d(c1.result(7).txnc()*Constants.MaxAngularRate))));*/

    driver
        .leftBumper()
        .and(vision.getCamera(TargetingComputer.currentTargetBranch.getPreferredCamera()).hasTarget) //get prefered camera
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    robotCentric
                        .withVelocityX(MaxSpeed.times(-vision.getCamera(TargetingComputer.currentTargetBranch.getPreferredCamera()).VelocityX(14)))
                        .withVelocityY(MaxSpeed.times(-driver.customLeft().getX()))));

    // TargetingComputer.currentTargetBranch.getApriltag() - use for random april tag

    driver
        .rightBumper()
        .onTrue(
            drivetrain.applyRequest(
                () ->
										robotCentric
                        .withVelocityX(
                            MaxSpeed.times(
                                -vision.getCamera(1).VelocityX(14)))
                        .withVelocityY(MaxSpeed.times(-driver.customLeft().getX()))
                        ));

    // TargetingComputer.currentTargetBranch.getApriltag() - use for random april tag

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    SwerveSetpointGen setpointGen =
        new SwerveSetpointGen(
                drivetrain.getChassisSpeeds(),
                drivetrain.getModuleStates(),
                drivetrain::getRotation)
            .withDeadband(MaxSpeed.times(0.025))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.025))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    driver
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                -driver.getLeftY())) // Drive forward with negative Y (forward)
                        .withVelocityY(MaxSpeed.times(-driver.getLeftX()))
                        .withRotationalRate(Constants.MaxAngularRate.times(-driver.getRightX()))
                        .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Custom Swerve Request that use ProfiledFieldCentricFacingAngle. Allows you to face specific
    // direction while driving
    ProfiledFieldCentricFacingAngle driveFacingAngle =
        new ProfiledFieldCentricFacingAngle(
                new TrapezoidProfile.Constraints(
                    Constants.MaxAngularRate.baseUnitMagnitude(),
                    Constants.MaxAngularRate.div(0.25).baseUnitMagnitude()))
            .withDeadband(MaxSpeed.times(0.1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Set PID for ProfiledFieldCentricFacingAngle
    driveFacingAngle.HeadingController.setPID(7, 0, 0);
    driver
        .y()
        .whileTrue(
            drivetrain
                .runOnce(() -> driveFacingAngle.resetProfile(drivetrain.getRotation()))
                .andThen(
                    drivetrain.applyRequest(
                        () ->
                            driveFacingAngle
                                .withVelocityX(
                                    MaxSpeed.times(
                                        -driver
                                            .getLeftY())) // Drive forward with negative Y (forward)
                                .withVelocityY(MaxSpeed.times(-driver.getLeftX()))
                                .withTargetDirection(
                                    new Rotation2d(-driver.getRightY(), -driver.getRightX())))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver.rightBumper().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.rightBumper().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.leftBumper().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.leftBumper().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driver
        .start()
        .onTrue(
            drivetrain.runOnce(
                () ->
                    drivetrain.resetPose(
                        new Pose2d(
                            drivetrain.getPose().getX(),
                            drivetrain.getPose().getY(),
                            new Rotation2d()))));

    // Mech Controller Bindings
    mech.y().onTrue(new InstantCommand(() -> TargetingComputer.setTargetLevel(4)));
    mech.x().onTrue(new InstantCommand(() -> TargetingComputer.setTargetLevel(3)));
    mech.a().onTrue(new InstantCommand(() -> TargetingComputer.setTargetLevel(2)));

    // Targeting Controller Bindings
    alphaButton
        .and(bravoButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.ALPHA)));
    bravoButton
        .and(alphaButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.BRAVO)));
    charlieButton
        .and(deltaButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.CHARLIE)));
    deltaButton
        .and(charlieButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.DELTA)));
    echoButton
        .and(foxtrotButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.ECHO)));
    foxtrotButton
        .and(echoButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.FOXTROT)));
    golfButton
        .and(hotelButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.GOLF)));
    hotelButton
        .and(golfButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.HOTEL)));
    indiaButton
        .and(julietButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.INDIA)));
    julietButton
        .and(indiaButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.JULIET)));
    kiloButton
        .and(limaButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.KILO)));
    limaButton
        .and(kiloButton.negate())
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.LIMA)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
