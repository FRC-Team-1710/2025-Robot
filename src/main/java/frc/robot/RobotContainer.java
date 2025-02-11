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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevationManual;
import frc.robot.commands.EndIntake;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.WristManual;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIO;
import frc.robot.subsystems.claw.ClawIOCTRE;
import frc.robot.subsystems.claw.ClawIOSIM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOCTRE;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.manipulator.ManipulatorIOTalonFX;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
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

  private final TunableController driver2 =
      new TunableController(1)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.125);

  private final TunableController reefTargetingSystem = new TunableController(2);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  public final Manipulator manipulator;
  public final Elevator elevator;
  public final Roller roller;
  public final Claw claw;
  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.025))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.025)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    roller = new Roller();
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);
        manipulator = new Manipulator(new ManipulatorIOTalonFX());
        elevator = new Elevator(new ElevatorIOCTRE());
        claw = new Claw(new ClawIOCTRE());

        new Vision(
            drivetrain::addVisionData,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);
        manipulator = new Manipulator(new ManipulatorIOSim());
        ElevatorIOSIM iosim = new ElevatorIOSIM();
        elevator = new Elevator(iosim);
        claw = new Claw(new ClawIOSIM(iosim));

        new Vision(
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
        manipulator = new Manipulator(new ManipulatorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        claw = new Claw(new ClawIO() {});

        new Vision(
            drivetrain::addVisionData,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});
        break;
    }

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
    claw.setDefaultCommand(new WristManual(claw, () -> driver2.getRightY()));
    elevator.setDefaultCommand(new ElevationManual(elevator, () -> driver.getLeftY()));
    driver.pov(0).onTrue(elevator.L4());
    driver.pov(180).onTrue(elevator.intake());
    driver.leftBumper().whileTrue(new PlaceCoral(manipulator));
    driver
        .rightBumper()
        .whileTrue(new IntakeCoral(manipulator, roller, driver))
        .onFalse(new EndIntake(manipulator, roller));

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

    // driver.a().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    SwerveSetpointGen setpointGen =
        new SwerveSetpointGen(
                drivetrain.getChassisSpeeds(),
                drivetrain.getModuleStates(),
                drivetrain::getRotation)
            .withDeadband(MaxSpeed.times(0.1))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
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

    reefTargetingSystem.a();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
