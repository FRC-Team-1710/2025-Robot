package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Mode;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevationManual;
import frc.robot.commands.ElevatorToTargetLevel;
import frc.robot.commands.EndIntake;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.IntakeForAuto;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.OutakeForAuto;
import frc.robot.commands.OuttakeCoral;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.StartClimb;
import frc.robot.commands.WristManual;
import frc.robot.commands.ZeroRizz;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.superstructure.LEDs.LEDSubsystem;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.subsystems.superstructure.claw.ClawIOCTRE;
import frc.robot.subsystems.superstructure.claw.ClawIOSIM;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOCTRE;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSIM;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.funnel.FunnelIO;
import frc.robot.subsystems.superstructure.funnel.FunnelIOCTRE;
import frc.robot.subsystems.superstructure.funnel.FunnelIOSIM;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorIO;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorIOCTRE;
import frc.robot.subsystems.superstructure.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TargetingComputer.Levels;
import frc.robot.utils.TargetingComputer.Targets;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController driver =
      new TunableController(0)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.1);

  private final TunableController mech =
      new TunableController(1)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.125);

  /** Port 5 */
  private final TunableController testing =
      new TunableController(5)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.125);

  private final Joystick reefTargetingSystem = new Joystick(2);

  private final TunableController sysID = new TunableController(3);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  public final Manipulator manipulator;
  public final Elevator elevator;
  public final Funnel funnel;
  public final Climber climber;
  public final Claw claw;
  public final LEDSubsystem ledsubsystem;

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

  /** Driver Y */
  private final Trigger placeL4 = new Trigger(driver.y());
  /** Driver X */
  private final Trigger placeL3 = new Trigger(driver.x());
  /** Driver A */
  private final Trigger placeL2 = new Trigger(driver.a());
  /** Driver B */
  private final Trigger grabAlgae = new Trigger(driver.b());
  /** Driver RT */

  /** Driver RT */
  private final Trigger targetReef = new Trigger(driver.rightTrigger());
  /** Driver LT */
  private final Trigger targetSource = new Trigger(driver.leftTrigger());
  /** Driver Left */
  private final Trigger previousTarget = new Trigger(driver.povLeft());
  /** Driver Right */
  private final Trigger nextTarget = new Trigger(driver.povRight());
  /** Driver Up */
  private final Trigger driverUp = new Trigger(driver.povUp());
  /** Driver Down */
  private final Trigger driverDown = new Trigger(driver.povDown());
  /** Driver Start */
  private final Trigger resetGyro = new Trigger(driver.start());
  /** Driver Back */
  private final Trigger prepClimb = new Trigger(driver.back());
  /** Driver RB */
  private final Trigger shootAlgae = new Trigger(driver.rightBumper());
  /** Driver LB */
  private final Trigger grabAlgaeFromFloor = new Trigger(driver.leftBumper());

  /** Mech Y */
  private final Trigger targetL4 = new Trigger(mech.y());
  /** Mech X */
  private final Trigger targetL3 = new Trigger(mech.x());
  /** Mech A */
  private final Trigger targetL2 = new Trigger(mech.a());
  /** Mech B */
  private final Trigger targetAlgae = new Trigger(mech.b());
  /** Mech Right */
  private final Trigger outtakeCoral = new Trigger(mech.povRight());
  /** Mech LB */
  private final Trigger bumpCoral = new Trigger(mech.leftBumper());
  /** Mech RB */
  private final Trigger intakeCoral = new Trigger(mech.rightBumper());
  /** Mech RT */
  private final Trigger climberUp = new Trigger(mech.rightTrigger());
  /** Mech LT */
  private final Trigger climberDown = new Trigger(mech.leftTrigger());
  /** Mech Left */
  private final Trigger overrideTargetingController = new Trigger(mech.povLeft());
  // Owen: "So we're like fourth cousins?" Micah: "It's far enough that you could
  // marry."

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
  private final Trigger l4Button = new Trigger(() -> reefTargetingSystem.getY() < -.5);
  private final Trigger l3Button = new Trigger(() -> reefTargetingSystem.getY() > .5);
  private final Trigger l2Button = new Trigger(() -> reefTargetingSystem.getX() > .5);
  private final Trigger l1Button = new Trigger(() -> reefTargetingSystem.getX() < -.5);

  public RobotContainer() {
    climber = new Climber();
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);
        manipulator = new Manipulator(new ManipulatorIOCTRE());
        elevator = new Elevator(new ElevatorIOCTRE());
        claw = new Claw(new ClawIOCTRE());
        funnel = new Funnel(new FunnelIOCTRE());
        ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);

        /*
         * Vision Class for referencing.
         * Contains 4 cameras as well as their respective names and standard deviations
         * from robot
         */
        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIOPhotonVision(
                    "FrontLeft",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(330)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "FrontRight",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            -Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(30)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackLeft",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(150)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            -Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(210)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters));
        vision.getCamera(0).useRejectionDistance(Constants.kCameraRejectionDistance); // Front Left
        vision.getCamera(1).useRejectionDistance(Constants.kCameraRejectionDistance); // Front Right
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);
        manipulator = new Manipulator(new ManipulatorIOSim());
        ElevatorIOSIM iosim = new ElevatorIOSIM();
        elevator = new Elevator(iosim);
        claw = new Claw(new ClawIOSIM(iosim));
        funnel = new Funnel(new FunnelIOSIM());
        ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIOPhotonVisionSIM(
                    "FrontLeft",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(330)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "FrontRight",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            -Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(30)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "BackLeft",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(150)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "BackRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            -Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(210)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters));

        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        claw = new Claw(new ClawIO() {});
        funnel = new Funnel(new FunnelIO() {});
        ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "Align to Alpha", drivetrain.Alignment(drive, Targets.ALPHA, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Bravo", drivetrain.Alignment(drive, Targets.BRAVO, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Charlie", drivetrain.Alignment(drive, Targets.CHARLIE, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Delta", drivetrain.Alignment(drive, Targets.DELTA, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Echo", drivetrain.Alignment(drive, Targets.ECHO, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Foxtrot", drivetrain.Alignment(drive, Targets.FOXTROT, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Golf", drivetrain.Alignment(drive, Targets.GOLF, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Hotel", drivetrain.Alignment(drive, Targets.HOTEL, vision, elevator));
    NamedCommands.registerCommand(
        "Align to India", drivetrain.Alignment(drive, Targets.INDIA, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Juliet", drivetrain.Alignment(drive, Targets.JULIET, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Kilo", drivetrain.Alignment(drive, Targets.KILO, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Lima", drivetrain.Alignment(drive, Targets.LIMA, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Source Right",
        drivetrain.Alignment(drive, Targets.SOURCE_RIGHT, vision, elevator));
    NamedCommands.registerCommand(
        "Align to Source Left", drivetrain.Alignment(drive, Targets.SOURCE_LEFT, vision, elevator));
    NamedCommands.registerCommand("intake coral", new IntakeForAuto(manipulator, funnel));
    NamedCommands.registerCommand(
        "outtake coral",
        new OutakeForAuto(elevator, manipulator, drivetrain, robotCentric)
            .alongWith(drivetrain.stop(robotCentric)));
    NamedCommands.registerCommand(
        "L2",
        elevator
            .L2()
            .alongWith(drivetrain.stop(robotCentric))
            .until(() -> elevator.isAtTarget())
            .andThen(new OutakeForAuto(elevator, manipulator, drivetrain, robotCentric)));
    NamedCommands.registerCommand(
        "L4",
        elevator
            .L4()
            .alongWith(drivetrain.stop(robotCentric))
            .until(() -> elevator.isAtTarget())
            .andThen(new OutakeForAuto(elevator, manipulator, drivetrain, robotCentric)));
    NamedCommands.registerCommand(
        "intake position",
        elevator
            .intake()
            .alongWith(drivetrain.stop(robotCentric))
            .onlyIf(() -> !manipulator.beam1Broken() && !manipulator.beam2Broken())
            .until(() -> elevator.isAtTarget()));

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
    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED.
    // Instructions
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

    double alignP = Constants.currentMode == Constants.Mode.SIM ? .75 : .8;
    double rotP = Constants.currentMode == Constants.Mode.SIM ? 1 : .4;

    claw.setDefaultCommand(new WristManual(claw, () -> mech.getRightY()));
    elevator.setDefaultCommand(new ElevationManual(elevator, () -> mech.getLeftY()));
    // driver
    // .rightBumper()
    // .whileTrue(new IntakeCoral(manipulator, funnel, driver, mech.leftBumper()))
    // .onFalse(new EndIntake(manipulator, funnel));

    driverUp
        .and(() -> Constants.currentMode == Constants.Mode.SIM)
        .onTrue(new InstantCommand(() -> claw.toggleAlgaeStatus()));
    driverDown
        .and(() -> Constants.currentMode == Constants.Mode.SIM)
        .onTrue(new InstantCommand(() -> manipulator.toggleCoralStatus()));

    driverUp.onTrue(funnel.L1()).onFalse(funnel.intake());

    new Trigger(() -> claw.hasAlgae())
        .onTrue(new InstantCommand(() -> TargetingComputer.updateSourceCutoffDistance(true)))
        .onFalse(
            claw.IDLE()
                .alongWith(
                    new InstantCommand(() -> TargetingComputer.setAligningWithAlgae(false))));

    new Trigger(() -> TargetingComputer.stillOuttakingAlgae)
        .onFalse(
            new InstantCommand(() -> TargetingComputer.updateSourceCutoffDistance(false))
                .unless(() -> claw.hasAlgae()));

    // new Trigger(() -> funnel.hasCoral())
    //     .onTrue(
    //         new InstantCommand(() -> funnel.setRollerPower(0))
    //             .onlyIf(() -> !elevator.isAtIntake())
    //             .alongWith(elevator.intake())
    //             .onlyIf(() -> !elevator.isAtIntake()))
    //     .and(() -> elevator.isAtIntake())
    //     .onTrue(
    //         new InstantCommand(() -> funnel.setRollerPower(FunnelConstants.intakeSpeed))
    //             .unless(() -> TargetingComputer.currentTargetLevel == Levels.L1));

    new Trigger(() -> drivetrain.isNearProcessor())
        .and(targetSource)
        .and(() -> claw.hasAlgae())
        .onTrue(claw.PROCESSOR())
        .onFalse(claw.IDLE());

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                setpointGen
                    .withVelocityX(
                        MaxSpeed.times(
                            -driver.customLeft().getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -driver.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(-driver.customRight().getX())
                            .times(
                                claw.hasAlgae()
                                    ? .5
                                    : 1)))); // Drive counterclockwise with negative X (left)

    // elevator.setDefaultCommand(new ElevationManual(elevator, () ->
    // mech.getLeftY()));

    /* Driver Bindings */
    placeL2
        .onTrue(
            new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L2))
                .alongWith(new ElevatorToTargetLevel(elevator)))
        .onFalse(elevator.intake().unless(targetReef))
        .and(
            () ->
                Math.abs(
                            new Rotation2d(
                                    Units.degreesToRadians(
                                        TargetingComputer.getCurrentTargetBranch()
                                            .getTargetingAngle()))
                                .minus(drivetrain.getPose().getRotation())
                                .getDegrees())
                        < TargetingComputer.alignmentAngleTolerance
                    // && drivetrain
                    //         .getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose())
                    //         .getNorm()
                    //     < TargetingComputer.alignmentTranslationTolerance
                    && targetReef.getAsBoolean()
                    && !TargetingComputer.targetingAlgae)
        .whileTrue(new PlaceCoral(elevator, manipulator, driver));

    placeL3
        .onTrue(
            new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L3))
                .alongWith(new ElevatorToTargetLevel(elevator)))
        .onFalse(elevator.intake().unless(targetReef))
        .and(
            () ->
                Math.abs(
                            new Rotation2d(
                                    Units.degreesToRadians(
                                        TargetingComputer.getCurrentTargetBranch()
                                            .getTargetingAngle()))
                                .minus(drivetrain.getPose().getRotation())
                                .getDegrees())
                        < TargetingComputer.alignmentAngleTolerance
                    // && drivetrain
                    //         .getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose())
                    //         .getNorm()
                    //     < TargetingComputer.alignmentTranslationTolerance
                    && targetReef.getAsBoolean()
                    && !TargetingComputer.targetingAlgae)
        .whileTrue(new PlaceCoral(elevator, manipulator, driver));

    placeL4
        .onTrue(
            new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L4))
                .alongWith(new ElevatorToTargetLevel(elevator))
                .alongWith(
                    claw.NET()
                        .onlyIf(
                            () ->
                                TargetingComputer.getSourceTargetingAngle(drivetrain.getPose())
                                        == TargetingComputer.Targets.NET.getTargetingAngle()
                                    && targetSource.getAsBoolean())))
        .onFalse(elevator.intake().unless(targetReef).alongWith(claw.IDLE()))
        .and(
            () ->
                Math.abs(
                            new Rotation2d(
                                    Units.degreesToRadians(
                                        TargetingComputer.getCurrentTargetBranch()
                                            .getTargetingAngle()))
                                .minus(drivetrain.getPose().getRotation())
                                .getDegrees())
                        < TargetingComputer.alignmentAngleTolerance
                    // && drivetrain
                    //         .getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose())
                    //         .getNorm()
                    //     < TargetingComputer.alignmentTranslationTolerance
                    && targetReef.getAsBoolean()
                    && !TargetingComputer.targetingAlgae)
        .whileTrue(new PlaceCoral(elevator, manipulator, driver));

    placeL4
        .and(
            () ->
                elevator.isAtTarget()
                    && claw.hasAlgae()
                    && (!targetReef.getAsBoolean() || TargetingComputer.targetingAlgae))
        .onTrue(claw.NET()); // TODO: fix

    grabAlgae
        .onFalse(
            new InstantCommand(() -> claw.lockRoller())
                .alongWith(elevator.intake().unless(targetReef))
                .alongWith(new ElevatorToTargetLevel(elevator).unless(targetReef.negate())))
        .and(() -> !claw.hasAlgae() && !TargetingComputer.stillOuttakingAlgae)
        .onFalse(
            new InstantCommand(() -> TargetingComputer.setAligningWithAlgae(false))
                .unless(() -> targetReef.getAsBoolean() || claw.hasAlgae())
                .alongWith(new InstantCommand(() -> TargetingComputer.setReadyToGrabAlgae(false)))
                .alongWith(claw.IDLE()))
        .onTrue(
            new InstantCommand(() -> TargetingComputer.setAligningWithAlgae(true))
                .alongWith(new InstantCommand(() -> TargetingComputer.setTargetingAlgae(true)))
                .alongWith(
                    new ElevatorToTargetLevel(elevator)
                        .alongWith(claw.GRAB().alongWith(new GrabAlgae(claw)))
                        .onlyIf(() -> drivetrain.isInAlignmentZone() && targetReef.getAsBoolean()))
                .unless(
                    () ->
                        TargetingComputer.getSourceTargetingAngle(drivetrain.getPose())
                            == TargetingComputer.Targets.NET.getTargetingAngle()));

    grabAlgae
        .and(targetReef)
        .and(
            () ->
                !claw.hasAlgae()
                    && !TargetingComputer.stillOuttakingAlgae
                    && drivetrain.isInAlignmentZone())
        .onTrue(claw.GRAB().alongWith(new GrabAlgae(claw)));

    grabAlgae
        .and(() -> !claw.hasAlgae())
        .and(
            () ->
                Math.abs(
                            new Rotation2d(
                                    Units.degreesToRadians(
                                        TargetingComputer.getCurrentTargetBranch()
                                            .getTargetingAngle()))
                                .minus(drivetrain.getPose().getRotation())
                                .getDegrees())
                        < TargetingComputer.alignmentAngleTolerance
                    && drivetrain
                            .getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose())
                            .getNorm()
                        < TargetingComputer.alignmentTranslationTolerance
                    && elevator.isAtTarget()
                    && claw.isAtTarget()
                    && claw.getMode() == Claw.ClawPosition.GRAB)
        .onTrue(new InstantCommand(() -> TargetingComputer.setReadyToGrabAlgae(true)));

    grabAlgae
        .and(
            () ->
                claw.hasAlgae()
                    && !targetReef.getAsBoolean()
                    && TargetingComputer.getSourceTargetingAngle(drivetrain.getPose())
                        != TargetingComputer.Targets.NET.getTargetingAngle())
        .onTrue(claw.PROCESSOR())
        .onFalse(claw.IDLE());

    shootAlgae
        .and((() -> claw.getMode() != Claw.ClawPosition.PROCESSOR))
        .onTrue(new InstantCommand(() -> claw.setRollers(-.5)))
        .onFalse(new InstantCommand(() -> claw.setRollers(0)))
        .and(() -> Constants.currentMode == Mode.SIM)
        .onTrue(new InstantCommand(() -> claw.setAlgaeStatus(false)));

    shootAlgae
        .and((() -> claw.getMode() == Claw.ClawPosition.PROCESSOR))
        .onTrue(new InstantCommand(() -> claw.setRollers(-.25)))
        .onFalse(new InstantCommand(() -> claw.setRollers(0)))
        .and(() -> Constants.currentMode == Mode.SIM)
        .onTrue(new InstantCommand(() -> claw.setAlgaeStatus(false)));

    grabAlgaeFromFloor.onTrue(claw.PROCESSOR()).whileTrue(new GrabAlgae(claw)).onFalse(claw.IDLE());

    previousTarget
        .and(() -> TargetingComputer.targetingControllerOverride ? targetReef.getAsBoolean() : true)
        .onTrue(
            new InstantCommand(
                () ->
                    TargetingComputer.setTargetBranch(
                        TargetingComputer.getTargetFromGameID(
                            TargetingComputer.getCurrentTargetBranch().gameID - 1))));

    nextTarget
        .and(() -> TargetingComputer.targetingControllerOverride ? targetReef.getAsBoolean() : true)
        .onTrue(
            new InstantCommand(
                () ->
                    TargetingComputer.setTargetBranch(
                        TargetingComputer.getTargetFromGameID(
                            TargetingComputer.getCurrentTargetBranch().gameID + 1))));

    // driver
    // .b()
    // .whileTrue(
    // drivetrain.applyRequest(
    // () ->
    // point.withModuleDirection(
    // new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // AprilTag Alignment

    targetReef // Get the closest reef AprilTag and set it to the target tag
        .onTrue(
        new InstantCommand(() -> vision.autoBranchTargeting()) // Set the target side/AprilTag
            .alongWith( // Log the curent target branch for debugging
                new InstantCommand(
                    () ->
                        Logger.recordOutput(
                            "Overide Target", TargetingComputer.getCurrentTargetBranch())))
            .alongWith( // Makes sure that the target hasn't been set officially (won't instantly
                // move to the set target)
                new InstantCommand(() -> TargetingComputer.setTargetSet(false))));

    targetReef // Only rotate the robot when the d-pads haven't sent a signal
        .onFalse(
            elevator
                .intake()
                .unless(placeL4.or(placeL3).or(placeL2).or(grabAlgae))
                .alongWith(
                    new InstantCommand(() -> TargetingComputer.setAligningWithAlgae(false))
                        .unless(() -> claw.hasAlgae())))
        .and(
            () ->
                !drivetrain.isInAlignmentZone()
                    || Math.abs(
                            new Rotation2d(
                                    Units.degreesToRadians(
                                        TargetingComputer.getCurrentTargetBranch()
                                            .getTargetingAngle()))
                                .minus(drivetrain.getPose().getRotation())
                                .getDegrees())
                        > 5)
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                -driver
                                    .customLeft()
                                    .getY())) // Drive forward with negative Y (forward)
                        .withVelocityY(MaxSpeed.times(-driver.customLeft().getX()))
                        .withRotationalRate(
                            Constants.MaxAngularRate.times(
                                (new Rotation2d(
                                                        Units.degreesToRadians(
                                                            TargetingComputer
                                                                .getCurrentTargetBranch()
                                                                .getTargetingAngle()))
                                                    .minus(drivetrain.getPose().getRotation())
                                                    .getRadians())
                                                * rotP
                                            > .5
                                        && claw.hasAlgae()
                                    ? .5
                                    : (new Rotation2d(
                                                Units.degreesToRadians(
                                                    TargetingComputer.getCurrentTargetBranch()
                                                        .getTargetingAngle())))
                                            .minus(drivetrain.getPose().getRotation())
                                            .getRadians()
                                        * rotP))));

    // targetReef // Allows the robot to start moving and also sets whether the left
    // side is true or
    // // not
    // .and(() -> previousTarget.getAsBoolean() || nextTarget.getAsBoolean())
    // .onTrue(
    // new InstantCommand(() -> TargetingComputer.setTargetSet(true))
    // .alongWith(
    // new InstantCommand(
    // () -> vision.autoBranchTargeting(previousTarget.getAsBoolean()))));

    targetReef // Robot moves to the current target
        .and(
            () ->
                drivetrain.isInAlignmentZone()
                    && Math.abs(
                            new Rotation2d(
                                    Units.degreesToRadians(
                                        TargetingComputer.getCurrentTargetBranch()
                                            .getTargetingAngle()))
                                .minus(drivetrain.getPose().getRotation())
                                .getDegrees())
                        < TargetingComputer.alignmentAngleTolerance)
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                    ((TargetingComputer.getCurrentTargetBranchPose().getX()
                                                    - drivetrain.getPose().getX()))
                                                * alignP
                                            > TargetingComputer.maxAlignSpeed
                                        ? TargetingComputer.maxAlignSpeed
                                        : (TargetingComputer.getCurrentTargetBranchPose().getX()
                                                - drivetrain.getPose().getX())
                                            * alignP)
                                .times(Robot.getAlliance() ? -1 : 1))
                        .withVelocityY(
                            MaxSpeed.times(
                                    (TargetingComputer.getCurrentTargetBranchPose().getY()
                                                    - drivetrain.getPose().getY())
                                                * alignP
                                            > TargetingComputer.maxAlignSpeed
                                        ? TargetingComputer.maxAlignSpeed
                                        : (TargetingComputer.getCurrentTargetBranchPose().getY()
                                                - drivetrain.getPose().getY())
                                            * alignP)
                                .times(Robot.getAlliance() ? -1 : 1))
                        .withRotationalRate(
                            Constants.MaxAngularRate.times(
                                (new Rotation2d(
                                                        Units.degreesToRadians(
                                                            TargetingComputer
                                                                .getCurrentTargetBranch()
                                                                .getTargetingAngle()))
                                                    .minus(drivetrain.getPose().getRotation())
                                                    .getRadians())
                                                * rotP
                                            > .5
                                        && claw.hasAlgae()
                                    ? .5
                                    : (new Rotation2d(
                                                Units.degreesToRadians(
                                                    TargetingComputer.getCurrentTargetBranch()
                                                        .getTargetingAngle())))
                                            .minus(drivetrain.getPose().getRotation())
                                            .getRadians()
                                        * rotP))))
        .onFalse(
            elevator
                .intake()
                .unless(
                    () ->
                        placeL4.getAsBoolean()
                            || placeL3.getAsBoolean()
                            || placeL2.getAsBoolean()
                            || grabAlgae.getAsBoolean()
                            || targetReef.getAsBoolean()
                            || targetAlgae.getAsBoolean())
                .alongWith(claw.IDLE())
                .alongWith(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0))))
        .onTrue(new ElevatorToTargetLevel(elevator))
        .and(
            () ->
                drivetrain
                            .getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose())
                            .getNorm()
                        < TargetingComputer.alignmentTranslationTolerance
                    && elevator.isAtTarget()
                    && manipulator.hasCoral())
        .onTrue(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, .2)))
        .onFalse(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0)));

    targetSource
        .onFalse(new InstantCommand(() -> TargetingComputer.setStillOuttakingAlgae(false)))
        .and(targetReef.negate())
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                -driver
                                    .customLeft()
                                    .getY())) // Drive forward with negative Y (forward)
                        .withVelocityY(MaxSpeed.times(-driver.customLeft().getX()))
                        .withRotationalRate(
                            Constants.MaxAngularRate.times(
                                (new Rotation2d(
                                                        Units.degreesToRadians(
                                                            TargetingComputer
                                                                .getSourceTargetingAngle(
                                                                    drivetrain.getPose())))
                                                    .minus(drivetrain.getPose().getRotation())
                                                    .getRadians())
                                                * rotP
                                            > .5
                                        && claw.hasAlgae()
                                    ? .5
                                    : (new Rotation2d(
                                                Units.degreesToRadians(
                                                    TargetingComputer.getSourceTargetingAngle(
                                                        drivetrain.getPose()))))
                                            .minus(drivetrain.getPose().getRotation())
                                            .getRadians()
                                        * rotP))))
        .and(
            () ->
                !TargetingComputer.stillOuttakingAlgae
                    || TargetingComputer.stillInRangeOfSources(drivetrain.getPose())
                    || !TargetingComputer.goForClimb)
        .whileTrue(
            new IntakeCoral(manipulator, funnel, driver, mech.leftBumper())
                .unless(() -> TargetingComputer.currentTargetLevel == Levels.L1))
        .onFalse(
            new EndIntake(manipulator, funnel, mech.leftBumper())
                .unless(() -> TargetingComputer.currentTargetLevel == Levels.L1));

    targetSource
        .and(() -> claw.hasAlgae())
        .onTrue(new InstantCommand(() -> TargetingComputer.setStillOuttakingAlgae(true)))
        .and(
            () ->
                TargetingComputer.getSourceTargetingAngle(drivetrain.getPose())
                    == TargetingComputer.Targets.NET.getTargetingAngle())
        .and(grabAlgae)
        .onTrue(elevator.L4().alongWith(claw.NET()))
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                    ((FieldConstants.aprilTags
                                                        .getTagPose(
                                                            TargetingComputer.Targets.NET
                                                                .getApriltag())
                                                        .get()
                                                        .getX()
                                                    + (Robot.getAlliance()
                                                        ? TargetingComputer.Targets.NET
                                                            .getOffset()
                                                            .getX()
                                                        : -TargetingComputer.Targets.NET
                                                            .getOffset()
                                                            .getX())
                                                    - drivetrain.getPose().getX()))
                                                * alignP
                                            > TargetingComputer.maxAlignSpeed
                                        ? TargetingComputer.maxAlignSpeed
                                        : (FieldConstants.aprilTags
                                                    .getTagPose(
                                                        TargetingComputer.Targets.NET.getApriltag())
                                                    .get()
                                                    .getX()
                                                + (Robot.getAlliance()
                                                    ? TargetingComputer.Targets.NET
                                                        .getOffset()
                                                        .getX()
                                                    : -TargetingComputer.Targets.NET
                                                        .getOffset()
                                                        .getX())
                                                - drivetrain.getPose().getX())
                                            * alignP)
                                .times(Robot.getAlliance() ? -1 : 1))
                        .withVelocityY(MaxSpeed.times(-driver.customLeft().getX() * .5))
                        .withRotationalRate(
                            Constants.MaxAngularRate.times(
                                (new Rotation2d(
                                                        Units.degreesToRadians(
                                                            TargetingComputer
                                                                .getSourceTargetingAngle(
                                                                    drivetrain.getPose())))
                                                    .minus(drivetrain.getPose().getRotation())
                                                    .getRadians())
                                                * rotP
                                            > .5
                                        && claw.hasAlgae()
                                    ? .5
                                    : (new Rotation2d(
                                                Units.degreesToRadians(
                                                    TargetingComputer.getSourceTargetingAngle(
                                                        drivetrain.getPose()))))
                                            .minus(drivetrain.getPose().getRotation())
                                            .getRadians()
                                        * rotP))));

    sysID
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                -sysID.getLeftY())) // Drive forward with negative Y (forward)
                        .withVelocityY(MaxSpeed.times(-sysID.getLeftX()))
                        .withRotationalRate(Constants.MaxAngularRate.times(-sysID.getRightX()))
                        .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single

    sysID.rightBumper().and(sysID.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    sysID.rightBumper().and(sysID.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    sysID.leftBumper().and(sysID.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    sysID.leftBumper().and(sysID.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
                            new Rotation2d(Robot.getAlliance() ? Math.PI : 0)))));

    // driver.a().onTrue(Commands.runOnce(() ->
    // drivetrain.resetPose(Pose2d.kZero)));

    prepClimb.onTrue(
        new InstantCommand(() -> TargetingComputer.setGoForClimb(true)).alongWith(funnel.CLIMB()));

    /* Mech Controller Bindings */
    targetL4.onTrue(
        new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L4)));
    targetL3.onTrue(
        new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L3)));
    targetL2.onTrue(
        new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L2)));
    targetAlgae.onTrue(
        new InstantCommand(
            () ->
                TargetingComputer.setTargetLevel(
                    TargetingComputer.getCurrentTargetBranch().getAlgaeLevel())));

    mech.pov(0).onTrue(new StartClimb(climber));
    mech.pov(180).onTrue(new Climb(climber));

    bumpCoral
        .onTrue(
            new InstantCommand(() -> funnel.setRollerPower(-0.075))
                .alongWith(new InstantCommand(() -> manipulator.runPercent(-0.075))))
        .onFalse(
            new EndIntake(manipulator, funnel, mech.leftBumper())
                .unless(() -> TargetingComputer.currentTargetLevel == Levels.L1));

    mech.start().whileTrue(new ZeroRizz(claw));

    mech.back().onTrue(new InstantCommand(() -> elevator.zero()));

    outtakeCoral.whileTrue(new OuttakeCoral(manipulator));
    intakeCoral
        .whileTrue(new IntakeCoral(manipulator, funnel, driver, mech.leftBumper()))
        .onFalse(new EndIntake(manipulator, funnel, mech.leftBumper()));

    overrideTargetingController.onTrue(
        new InstantCommand(TargetingComputer::toggleTargetingControllerOverride));

    climberUp.whileTrue(new ManualClimb(climber, () -> .2));
    climberDown.whileTrue(new ManualClimb(climber, () -> -.2).onlyIf(() -> climber.safeToRetract));

    /* Targeting Controller Bindings */
    alphaButton
        .and(bravoButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.ALPHA)));
    bravoButton
        .and(alphaButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.BRAVO)));
    charlieButton
        .and(deltaButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.CHARLIE)));
    deltaButton
        .and(charlieButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.DELTA)));
    echoButton
        .and(foxtrotButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.ECHO)));
    foxtrotButton
        .and(echoButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.FOXTROT)));
    golfButton
        .and(hotelButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.GOLF)));
    hotelButton
        .and(golfButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.HOTEL)));
    indiaButton
        .and(julietButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.INDIA)));
    julietButton
        .and(indiaButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.JULIET)));
    kiloButton
        .and(limaButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.KILO)));
    limaButton
        .and(kiloButton.negate())
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.LIMA)));
    l4Button
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetLevel(Levels.L4)));
    l3Button
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetLevel(Levels.L3)));
    l2Button
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetLevel(Levels.L2)));
    l1Button
        .and(() -> !TargetingComputer.targetingControllerOverride)
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetLevel(Levels.L1)));

    /* SysID Bindings */
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single

    sysID.rightBumper().and(sysID.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    sysID.rightBumper().and(sysID.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    sysID.leftBumper().and(sysID.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    sysID.leftBumper().and(sysID.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Temp elevator tuning :)

    testing.a().onTrue(elevator.intake());
    testing.b().onTrue(elevator.L2());
    testing.x().onTrue(elevator.L3());
    testing.y().onTrue(elevator.L4());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
