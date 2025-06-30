package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ReefFaces;
import frc.robot.subsystems.Superstructure.ReefLevel;
import frc.robot.subsystems.Superstructure.ReefSide;
import frc.robot.subsystems.Superstructure.TargetSourceSide;
import frc.robot.subsystems.Superstructure.WantedState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
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
import frc.robot.subsystems.vision.VisionIOAlgae;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final TunableController driver = new TunableController(0)
            .withControllerType(TunableControllerType.QUADRATIC)
            .withOutputAtDeadband(0.025)
            .withDeadband(0.1);

    private final TunableController mech = new TunableController(1)
            .withControllerType(TunableControllerType.QUADRATIC)
            .withOutputAtDeadband(0.025)
            .withDeadband(0.125);

    private final Joystick reefTargetingSystem = new Joystick(2);

    private final LoggedDashboardChooser<Command> autoChooser;

    public final Drive drivetrain;
    public final Manipulator manipulator;
    public final Elevator elevator;
    public final Funnel funnel;
    public final Climber climber;
    public final Claw claw;
    public final LEDSubsystem ledsubsystem;
    public final VisionIOAlgae algaeCam;

    public final Superstructure superstructure;

    private Vision vision;

    private final Trigger botWaitingForInput;

    private final Trigger endgame = new Trigger(
            () -> DriverStation.getMatchTime() < 30
                    && DriverStation.getMatchTime() > 25
                    && !DriverStation.isAutonomous());

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
                drivetrain = new Drive(currentDriveTrain);
                manipulator = new Manipulator(new ManipulatorIOCTRE());
                elevator = new Elevator(new ElevatorIOCTRE());
                claw = new Claw(new ClawIOCTRE());
                funnel = new Funnel(new FunnelIOCTRE());
                ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);
                algaeCam = new VisionIOAlgae("AlgaeCam");
                vision = new Vision(
                        drivetrain::addVisionData,
                        new VisionIOAlgae("AlgaeCam"),
                        new VisionIOPhotonVision(
                                "FrontLeft",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(330))),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVision(
                                "FrontRight",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                -Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(30))),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVision(
                                "BackLeft",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(150))),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVision(
                                "BackRight",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                -Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(210))),
                                drivetrain::getVisionParameters));
                vision.getCamera(0).useRejectionDistance(Constants.kCameraRejectionDistance);
                vision.getCamera(1).useRejectionDistance(Constants.kCameraRejectionDistance);

                break;

            case SIM:
                drivetrain = new Drive(currentDriveTrain);
                manipulator = new Manipulator(new ManipulatorIOSim());
                ElevatorIOSIM iosim = new ElevatorIOSIM();
                elevator = new Elevator(iosim);
                claw = new Claw(new ClawIOSIM(iosim));
                funnel = new Funnel(new FunnelIOSIM());
                ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);
                algaeCam = new VisionIOAlgae("AlgaeCam");

                vision = new Vision(
                        drivetrain::addVisionData,
                        new VisionIOAlgae("AlgaeCam"),
                        new VisionIOPhotonVisionSIM(
                                "FrontLeft",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(330))),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVisionSIM(
                                "FrontRight",
                                new Transform3d(
                                        new Translation3d(
                                                Units.inchesToMeters(10.066),
                                                -Units.inchesToMeters(11.959),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(30))),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVisionSIM(
                                "BackLeft",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(150))),
                                drivetrain::getVisionParameters),
                        new VisionIOPhotonVisionSIM(
                                "BackRight",
                                new Transform3d(
                                        new Translation3d(
                                                -Units.inchesToMeters(9.896),
                                                -Units.inchesToMeters(10.938),
                                                Units.inchesToMeters(8.55647482)),
                                        new Rotation3d(
                                                0,
                                                Units.degreesToRadians(-25.16683805),
                                                Units.degreesToRadians(210))),
                                drivetrain::getVisionParameters));

                break;

            default:
                drivetrain = new Drive(new DriveIO() {
                });
                manipulator = new Manipulator(new ManipulatorIO() {
                });
                elevator = new Elevator(new ElevatorIO() {
                });
                claw = new Claw(new ClawIO() {
                });
                funnel = new Funnel(new FunnelIO() {
                });
                ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);
                algaeCam = new VisionIOAlgae("AlgaeCam");

                vision = new Vision(
                        drivetrain::addVisionData,
                        new VisionIOAlgae("AlgaeCam"),
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

        superstructure = new Superstructure(drivetrain, claw, climber, elevator, funnel, ledsubsystem, manipulator,
                vision, driver, mech);
        
        superstructure.setTarget(ReefFaces.ab, ReefSide.left);
        superstructure.setTargetLevel(ReefLevel.L4);
        superstructure.setTargetSourceSide(TargetSourceSide.MIDDLE);
        superstructure.setWantedState(WantedState.DEFAULT_STATE);

        botWaitingForInput = new Trigger(superstructure::isWaitingForInput);

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoFactory.buildAutos());

        configureBindings();
    }

    private void configureBindings() {
        endgame.onTrue(Commands.runOnce(() -> mech.setRumble(RumbleType.kBothRumble, 1)))
                .onFalse(Commands.runOnce(() -> mech.setRumble(RumbleType.kBothRumble, 0)));

        botWaitingForInput.onTrue(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0.5)))
                .onFalse(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0)));

        alphaButton
                .and(bravoButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ab, ReefSide.left)));
        bravoButton
                .and(alphaButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ab, ReefSide.right)));
        charlieButton
                .and(deltaButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.cd, ReefSide.left)));
        deltaButton
                .and(charlieButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.cd, ReefSide.right)));
        echoButton
                .and(foxtrotButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ef, ReefSide.left)));
        foxtrotButton
                .and(echoButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ef, ReefSide.right)));
        golfButton
                .and(hotelButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.gh, ReefSide.left)));
        hotelButton
                .and(golfButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.gh, ReefSide.right)));
        indiaButton
                .and(julietButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ij, ReefSide.left)));
        julietButton
                .and(indiaButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ij, ReefSide.right)));
        kiloButton
                .and(limaButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.kl, ReefSide.left)));
        limaButton
                .and(kiloButton.negate())
                .onTrue(Commands.runOnce(() -> superstructure.setTarget(ReefFaces.kl, ReefSide.right)));
        l4Button
                .onTrue(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L4)));
        l3Button
                .onTrue(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L3)));
        l2Button
                .onTrue(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L2)));
        l1Button
                .onTrue(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L1)));
    }

    public void setAlliance(boolean alliance) {
        superstructure.setAlliance(alliance);
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
