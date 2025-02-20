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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignmentForAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EndIntake;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.OuttakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
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
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TargetingComputer.Targets;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  TargetingComputer targetingComputer = new TargetingComputer();

  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController driver =
      new TunableController(0)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.125);

  private final TunableController mech =
      new TunableController(1)
          .withControllerType(TunableControllerType.QUADRATIC)
          .withOutputAtDeadband(0.025)
          .withDeadband(0.125);

  private final Joystick reefTargetingSystem = new Joystick(2);

  private final LoggedDashboardChooser<Command> autoChooser;

  VisionIOPhotonVision c1;

  public final Drive drivetrain;
  public final Manipulator manipulator;
  public final Elevator elevator;
  public final Roller roller;
  public final Climber climber;
  // public final Claw claw;

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

  private final Trigger targetReef = new Trigger(driver.rightTrigger());

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

  public RobotContainer() {
    // claw = new Claw();
    climber = new Climber();
    roller = new Roller();
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);
        manipulator = new Manipulator(new ManipulatorIOTalonFX());
        elevator = new Elevator(new ElevatorIOCTRE());

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
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);
        manipulator = new Manipulator(new ManipulatorIOSim());
        elevator = new Elevator(new ElevatorIOSIM());

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

        // NamedCommands.registerCommand(
        //     "Align to Alpha", new AlignmentForAuto(vision, drivetrain, Targets.ALPHA));
        // NamedCommands.registerCommand(
        //     "Align to Bravo", new AlignmentForAuto(vision, drivetrain, Targets.BRAVO));
        NamedCommands.registerCommand(
            "Align to Charlie", new AlignmentForAuto(vision, drivetrain, Targets.CHARLIE));
        NamedCommands.registerCommand(
            "Align to Delta", new AlignmentForAuto(vision, drivetrain, Targets.DELTA));
        NamedCommands.registerCommand(
            "Align to Echo", new AlignmentForAuto(vision, drivetrain, Targets.ECHO));
        NamedCommands.registerCommand(
            "Align to Foxtrot", new AlignmentForAuto(vision, drivetrain, Targets.FOXTROT));
        NamedCommands.registerCommand(
            "Align to Golf", new AlignmentForAuto(vision, drivetrain, Targets.GOLF));
        NamedCommands.registerCommand(
            "Align to Hotel", new AlignmentForAuto(vision, drivetrain, Targets.HOTEL));
        NamedCommands.registerCommand(
            "Align to India", new AlignmentForAuto(vision, drivetrain, Targets.INDIA));
        NamedCommands.registerCommand(
            "Align to Juliet", new AlignmentForAuto(vision, drivetrain, Targets.JULIET));
        NamedCommands.registerCommand(
            "Align to Kilo", new AlignmentForAuto(vision, drivetrain, Targets.KILO));
        NamedCommands.registerCommand(
            "Align to Lima", new AlignmentForAuto(vision, drivetrain, Targets.LIMA));
        NamedCommands.registerCommand("Coral Intake", new IntakeCoral(manipulator, roller, driver));
        NamedCommands.registerCommand("outtake Coral", new OuttakeCoral(manipulator));
        NamedCommands.registerCommand(
            "Align to Bravo", drivetrain.Alignment(robotCentric, Targets.BRAVO, vision));

        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {});
        elevator = new Elevator(new ElevatorIO() {});

        vision =
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

    // elevator.setDefaultCommand(new ElevationManual(elevator, () -> mech.getLeftY()));
    driver.a().onTrue(elevator.L2()).onFalse(elevator.intake());
    driver.x().onTrue(elevator.L3()).onFalse(elevator.intake());
    driver.y().onTrue(elevator.L4()).onFalse(elevator.intake());
    driver.leftBumper().whileTrue(new OuttakeCoral(manipulator));
    driver
        .rightBumper()
        .whileTrue(new IntakeCoral(manipulator, roller, driver))
        .onFalse(new EndIntake(manipulator, roller));

    // driver.a().onTrue(new InstantCommand(() -> climber.SetClimberPower(0.1))).onFalse((new
    // InstantCommand(() -> climber.SetClimberPower(0))));
    // driver.b().onTrue(new InstantCommand(() -> climber.SetClimberPower(-0.1))).onFalse((new
    // InstantCommand(() -> climber.SetClimberPower(0))));

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            driver.customLeft().getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            driver.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -driver
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X (left)

    // driver.a().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

    // driver
    //     .povLeft()
    //     .whileTrue(
    //         drivetrain
    //             .applyRequest(
    //                 () ->
    //                     drive
    //                         .withVelocityX(
    //                             MaxSpeed.times(
    //                                 -driver
    //                                     .customLeft()
    //                                     .getY())) // Drive forward with negative Y (forward)
    //                         .withVelocityY(
    //                             MaxSpeed.times(
    //                                 -driver
    //                                     .customLeft()
    //                                     .getX())) // Drive left with negative X (left)
    //                         .withRotationalRate(Constants.MaxAngularRate.times(1)))
    //             .onlyIf(driver.rightTrigger().negate().or(driver.leftTrigger().negate())))
    //     .and(driver.rightTrigger())
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 TargetingComputer.setTargetBranch(
    //                     TargetingComputer.getTargetFromGameID(
    //                         TargetingComputer.getCurrentTargetBranch().gameID - 1))));

    // driver
    //     .povRight()
    //     .whileTrue(
    //         drivetrain
    //             .applyRequest(
    //                 () ->
    //                     drive
    //                         .withVelocityX(
    //                             MaxSpeed.times(
    //                                 -driver
    //                                     .customLeft()
    //                                     .getY())) // Drive forward with negative Y (forward)
    //                         .withVelocityY(
    //                             MaxSpeed.times(
    //                                 -driver
    //                                     .customLeft()
    //                                     .getX())) // Drive left with negative X (left)
    //                         .withRotationalRate(Constants.MaxAngularRate.times(-1)))
    //             .onlyIf(driver.rightTrigger().negate().or(driver.leftTrigger().negate())))
    //     .and(driver.rightTrigger())
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 TargetingComputer.setTargetBranch(
    //                     TargetingComputer.getTargetFromGameID(
    //                         TargetingComputer.getCurrentTargetBranch().gameID + 1))));

    driver
        .povLeft()
        .onTrue(
            new InstantCommand(
                () ->
                    TargetingComputer.setTargetBranch(
                        TargetingComputer.getTargetFromGameID(
                            TargetingComputer.getCurrentTargetBranch().gameID - 1))));

    driver
        .povRight()
        .onTrue(
            new InstantCommand(
                () ->
                    TargetingComputer.setTargetBranch(
                        TargetingComputer.getTargetFromGameID(
                            TargetingComputer.getCurrentTargetBranch().gameID + 1))));

    // driver
    //     .b()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    double alignP = 1;
    double rotP = .75;
    // AprilTag Alignment
    targetReef
        .and(
            () ->
                !vision.containsRequestedTarget(
                        TargetingComputer.getCurrentTargetBranch().getApriltag())
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
                    drive
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
                                                TargetingComputer.getCurrentTargetBranch()
                                                    .getTargetingAngle()))
                                        .minus(drivetrain.getPose().getRotation())
                                        .getRadians())
                                    * rotP))));
    targetReef
        .and(
            () ->
                vision.containsRequestedTarget(
                        TargetingComputer.getCurrentTargetBranch().getApriltag())
                    && Math.abs(
                            new Rotation2d(
                                    Units.degreesToRadians(
                                        TargetingComputer.getCurrentTargetBranch()
                                            .getTargetingAngle()))
                                .minus(drivetrain.getPose().getRotation())
                                .getDegrees())
                        < 5)
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    robotCentric
                        .withVelocityX(
                            MaxSpeed.times(
                                -(TargetingComputer.getCurrentTargetBranch().getOffset().getX()
                                        - vision
                                            .calculateOffset(
                                                TargetingComputer.getCurrentTargetBranch()
                                                    .getApriltag(),
                                                TargetingComputer.getCurrentTargetBranch()
                                                    .getOffset())
                                            .getX())
                                    * alignP))
                        .withVelocityY(
                            MaxSpeed.times(
                                -(TargetingComputer.getCurrentTargetBranch().getOffset().getY()
                                        - vision
                                            .calculateOffset(
                                                TargetingComputer.getCurrentTargetBranch()
                                                    .getApriltag(),
                                                TargetingComputer.getCurrentTargetBranch()
                                                    .getOffset())
                                            .getY())
                                    * alignP))
                        .withRotationalRate(
                            Constants.MaxAngularRate.times(
                                (new Rotation2d(
                                            Units.degreesToRadians(
                                                TargetingComputer.getCurrentTargetBranch()
                                                    .getTargetingAngle()))
                                        .minus(drivetrain.getPose().getRotation())
                                        .getRadians())
                                    * rotP))));

    driver
        .b()
        .onTrue(new InstantCommand(() -> TargetingComputer.setTargetingAlgae(true)))
        .onFalse(new InstantCommand(() -> TargetingComputer.setTargetingAlgae(false)));

    driver
        .leftTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
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
                                                TargetingComputer.getSourceTargetingAngle(
                                                    drivetrain.getPose())))
                                        .minus(drivetrain.getPose().getRotation())
                                        .getRadians())
                                    * rotP))));

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    // SwerveSetpointGen setpointGen =
    //     new SwerveSetpointGen(
    //             drivetrain.getChassisSpeeds(),
    //             drivetrain.getModuleStates(),
    //             drivetrain::getRotation)
    //         .withDeadband(MaxSpeed.times(0.025))
    //         .withRotationalDeadband(Constants.MaxAngularRate.times(0.025))
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // driver
    //     .x()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 setpointGen
    //                     .withVelocityX(
    //                         MaxSpeed.times(
    //                             -driver.getLeftY())) // Drive forward with negative Y (forward)
    //                     .withVelocityY(MaxSpeed.times(-driver.getLeftX()))
    //                     .withRotationalRate(Constants.MaxAngularRate.times(-driver.getRightX()))
    //
    // .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single

    mech.rightBumper().and(mech.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    mech.rightBumper().and(mech.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    mech.leftBumper().and(mech.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    mech.leftBumper().and(mech.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
