package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.StateHandler.States;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevationManual;
import frc.robot.commands.WristManual;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIO;
import frc.robot.subsystems.claw.ClawIOCTRE;
import frc.robot.subsystems.claw.ClawIOSIM;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOCTRE;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.manipulator.ManipulatorIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TargetingComputer.Levels;
import frc.robot.utils.TargetingComputer.Targets;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
    private final TunableController driver = new TunableController(0)
            .withControllerType(TunableControllerType.QUADRATIC)
            .withOutputAtDeadband(0.025)
            .withDeadband(0.125);

    private final TunableController mech = new TunableController(1)
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
    public final StateHandler stateHandler;

    // CTRE Default Drive Request
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.times(0.025))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.025)) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
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
    private final Trigger targetReef = new Trigger(driver.rightTrigger());
    /** Driver LT */
    private final Trigger targetSource = new Trigger(driver.leftTrigger());
    /** Driver RT */
    private final Trigger placeNet = new Trigger(driver.rightBumper());
    /** Driver LT */
    private final Trigger placeProcesser = new Trigger(driver.leftBumper());
    /** Driver Left */
    private final Trigger previousTarget = new Trigger(driver.povLeft());
    /** Driver Right */
    private final Trigger nextTarget = new Trigger(driver.povRight());
    /** Driver Start */
    private final Trigger resetGyro = new Trigger(driver.start());

    /** Mech Y */
    private final Trigger targetL4 = new Trigger(mech.y());
    /** Mech X */
    private final Trigger targetL3 = new Trigger(mech.x());
    /** Mech A */
    private final Trigger targetL2 = new Trigger(mech.a());
    /** Mech B */
    private final Trigger targetAlgae = new Trigger(mech.b());
    /** Mech LB */
    private final Trigger outtakeCoral = new Trigger(mech.leftBumper());
    /** Mech RB */
    private final Trigger intakeCoral = new Trigger(mech.rightBumper());
    /** Mech RB */
    private final Trigger resetState = new Trigger(mech.rightTrigger().and(mech.leftTrigger()));

    private final Trigger overrideTargetingController = new Trigger(mech.povDown());

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
        climber = new Climber();
        funnel = new Funnel();
        DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = new Drive(currentDriveTrain);
                manipulator = new Manipulator(new ManipulatorIOTalonFX());
                elevator = new Elevator(new ElevatorIOCTRE());
                claw = new Claw(new ClawIOCTRE());

                /*
                 * Vision Class for referencing.
                 * Contains 4 cameras as well as their respective names and standard deviations
                 * from robot
                 */
                vision = new Vision(
                        drivetrain::addVisionData,
                        new VisionIOPhotonVision(
                                "FrontLeft",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(330)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVision(
                                "FrontRight",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                -Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(30)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVision(
                                "BackLeft",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(150)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVision(
                                "BackRight",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                -Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(210)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain = new Drive(currentDriveTrain);
                manipulator = new Manipulator(new ManipulatorIOSim());
                ElevatorIOSIM iosim = new ElevatorIOSIM();
                elevator = new Elevator(iosim);
                claw = new Claw(new ClawIOSIM(iosim));

                vision = new Vision(
                        drivetrain::addVisionData,
                        new VisionIOPhotonVisionSIM(
                                "FrontLeft",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(330)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVisionSIM(
                                "FrontRight",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                -Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(30)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVisionSIM(
                                "BackLeft",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(150)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVisionSIM(
                                "BackRight",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                -Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)), // IN
                                        // METERS
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(210)) // IN
                                // RADIANS
                                ),
                                drivetrain::getVisionParameters));
                break;

            default:
                // Replayed robot, disable IO implementations
                drivetrain = new Drive(new DriveIO() {
                });
                manipulator = new Manipulator(new ManipulatorIO() {
                });
                elevator = new Elevator(new ElevatorIO() {
                });
                claw = new Claw(new ClawIO() {
                });

                vision = new Vision(
                        drivetrain::addVisionData,
                        new VisionIO() {
                        },
                        new VisionIO() {
                        },
                        new VisionIO() {
                        },
                        new VisionIO() {
                        });
                break;
        }

        stateHandler = new StateHandler(
                elevator, claw, climber, drivetrain, funnel, manipulator, vision, driver, mech);
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
        claw.setDefaultCommand(new WristManual(claw, () -> mech.getRightY()));

        elevator.setDefaultCommand(new ElevationManual(elevator, () -> mech.getLeftY()));

        resetState.onTrue(new InstantCommand(() -> stateHandler.resetToIdle()));

        resetGyro.onTrue(
                drivetrain.runOnce(
                        () -> drivetrain.resetPose(
                                new Pose2d(
                                        drivetrain.getPose().getX(),
                                        drivetrain.getPose().getY(),
                                        new Rotation2d()))));

        placeNet
                .onTrue(new InstantCommand(() -> stateHandler.schedualState(States.Net)))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.AlgaeIdle)));
        placeProcesser
                .onTrue(new InstantCommand(() -> stateHandler.schedualState(States.Processer)))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.AlgaeIdle)));

        placeNet
                .and(placeL2)
                .onTrue(new InstantCommand(() -> stateHandler.schedualState(States.PlaceAlgae)))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.Idle)));
        placeProcesser
                .and(placeL2)
                .onTrue(new InstantCommand(() -> stateHandler.schedualState(States.PlaceAlgae)))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.Idle)));
        placeL2.and(targetReef.negate()).and(placeProcesser.negate()).and(placeNet.negate())
                .onTrue(new InstantCommand(() -> stateHandler.schedualState(States.EjectAlgae)))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.Idle)));

        placeL4
                .and(targetReef)
                .onTrue(
                        new InstantCommand(() -> TargetingComputer.setTargetLevel(Levels.L4))
                                .alongWith(new InstantCommand(() -> stateHandler.schedualState(States.PlaceCoral))))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.PrepCoralPlace)));
        placeL3
                .and(targetReef)
                .onTrue(
                        new InstantCommand(() -> TargetingComputer.setTargetLevel(Levels.L3))
                                .alongWith(new InstantCommand(() -> stateHandler.schedualState(States.PlaceCoral))))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.PrepCoralPlace)));
        placeL2
                .and(targetReef)
                .onTrue(
                        new InstantCommand(() -> TargetingComputer.setTargetLevel(Levels.L2))
                                .alongWith(new InstantCommand(() -> stateHandler.schedualState(States.PlaceCoral))))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.PrepCoralPlace)));
        grabAlgae
                .and(targetReef)
                .onTrue(
                        new InstantCommand(() -> TargetingComputer.setTargetingAlgae(true))
                                .alongWith(
                                        new InstantCommand(() -> stateHandler.schedualState(States.PrepAlgaeGrab))));

        targetSource
                .and(targetReef.negate())
                .onTrue(new InstantCommand(() -> stateHandler.schedualState(States.IntakeFromSource)))
                .onFalse(new InstantCommand(() -> stateHandler.schedualState(States.Idle)));

        targetReef
                .onTrue(new InstantCommand(() -> stateHandler.schedualState(States.PrepCoralPlace)))
                .onFalse(
                        new InstantCommand(() -> stateHandler.schedualState(States.Idle))
                                .unless(targetSource)
                                .unless(() -> stateHandler.gettingAlgae()));

        // Targeting Computer Triggers//

        previousTarget.onTrue(
                new InstantCommand(
                        () -> TargetingComputer.setTargetBranch(
                                TargetingComputer.getTargetFromGameID(
                                        TargetingComputer.getCurrentTargetBranch().gameID - 1))));

        nextTarget.onTrue(
                new InstantCommand(
                        () -> TargetingComputer.setTargetBranch(
                                TargetingComputer.getTargetFromGameID(
                                        TargetingComputer.getCurrentTargetBranch().gameID + 1))));

        targetL4.onTrue(
                new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L4)));
        targetL3.onTrue(
                new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L3)));
        targetL2.onTrue(
                new InstantCommand(() -> TargetingComputer.setTargetLevel(TargetingComputer.Levels.L2)));
        targetAlgae.onTrue(
                new InstantCommand(
                        () -> TargetingComputer.setTargetLevel(
                                TargetingComputer.getCurrentTargetBranch().getAlgaeLevel())));

        overrideTargetingController.onTrue(
                new InstantCommand(() -> TargetingComputer.toggleTargetingControllerOverride()));

        alphaButton
                .and(bravoButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.ALPHA)));
        bravoButton
                .and(alphaButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.BRAVO)));
        charlieButton
                .and(deltaButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.CHARLIE)));
        deltaButton
                .and(charlieButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.DELTA)));
        echoButton
                .and(foxtrotButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.ECHO)));
        foxtrotButton
                .and(echoButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.FOXTROT)));
        golfButton
                .and(hotelButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.GOLF)));
        hotelButton
                .and(golfButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.HOTEL)));
        indiaButton
                .and(julietButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.INDIA)));
        julietButton
                .and(indiaButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.JULIET)));
        kiloButton
                .and(limaButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.KILO)));
        limaButton
                .and(kiloButton.negate())
                .and(() -> !TargetingComputer.targetingControllerOverride)
                .onTrue(new InstantCommand(() -> TargetingComputer.setTargetBranch(Targets.LIMA)));

        // /* SysID Bindings */
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single

        //
        // sysID.rightBumper().and(sysID.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //
        // sysID.rightBumper().and(sysID.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //
        // sysID.leftBumper().and(sysID.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //
        // sysID.leftBumper().and(sysID.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
