package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutosBuilder;
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

public class RobotContainer {
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

  private final Joystick reefTargetingSystem = new Joystick(2);

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

  private final AutosBuilder autosBuilder;

  private final Trigger endgame =
      new Trigger(
          () ->
              DriverStation.getMatchTime() < 30
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
        // drivetrain = new Drive(new DriveIO() {});
        drivetrain = new Drive(currentDriveTrain);
        manipulator =
            new Manipulator(new ManipulatorIOCTRE(), () -> driver.leftBumper().getAsBoolean());
        // elevator =
        // new Elevator(
        // new ElevatorIO() {},
        // () -> mech.getLeftY());
        elevator = new Elevator(new ElevatorIOCTRE(), () -> mech.getLeftY());
        claw = new Claw(new ClawIOCTRE(), () -> driver.leftBumper().getAsBoolean());
        funnel = new Funnel(new FunnelIOCTRE(), () -> driver.leftBumper().getAsBoolean());
        ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);
        algaeCam = new VisionIOAlgae("AlgaeCam");
        vision =
            new Vision(
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
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(330))),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "FrontRight",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            -Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)),
                        new Rotation3d(
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(30))),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackLeft",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)),
                        new Rotation3d(
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(150))),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            -Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)),
                        new Rotation3d(
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(210))),
                    drivetrain::getVisionParameters));
        vision.getCamera(0).useRejectionDistance(Constants.kCameraRejectionDistance);
        vision.getCamera(1).useRejectionDistance(Constants.kCameraRejectionDistance);

        break;

      case SIM:
        drivetrain = new Drive(currentDriveTrain);
        manipulator =
            new Manipulator(new ManipulatorIOSim(), () -> driver.leftBumper().getAsBoolean());
        ElevatorIOSIM iosim = new ElevatorIOSIM();
        elevator = new Elevator(iosim, () -> mech.getLeftY());
        claw = new Claw(new ClawIOSIM(iosim), () -> driver.leftBumper().getAsBoolean());
        funnel = new Funnel(new FunnelIOSIM(), () -> driver.leftBumper().getAsBoolean());
        ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);
        algaeCam = new VisionIOAlgae("AlgaeCam");

        vision =
            new Vision(
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
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(330))),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "FrontRight",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            -Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)),
                        new Rotation3d(
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(30))),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "BackLeft",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)),
                        new Rotation3d(
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(150))),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "BackRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            -Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)),
                        new Rotation3d(
                            0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(210))),
                    drivetrain::getVisionParameters));

        break;

      default:
        drivetrain = new Drive(new DriveIO() {});
        manipulator =
            new Manipulator(new ManipulatorIO() {}, () -> driver.leftBumper().getAsBoolean());
        elevator = new Elevator(new ElevatorIO() {}, () -> mech.getLeftY());
        claw = new Claw(new ClawIO() {}, () -> driver.leftBumper().getAsBoolean());
        funnel = new Funnel(new FunnelIO() {}, () -> driver.leftBumper().getAsBoolean());
        ledsubsystem = new LEDSubsystem(funnel, manipulator, climber, elevator, drivetrain);
        algaeCam = new VisionIOAlgae("AlgaeCam");

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIOAlgae("AlgaeCam"),
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    superstructure =
        new Superstructure(
            drivetrain,
            claw,
            climber,
            elevator,
            funnel,
            ledsubsystem,
            manipulator,
            vision,
            driver,
            mech);

    superstructure.setTarget(ReefFaces.ab, ReefSide.left);
    superstructure.setTargetLevel(ReefLevel.L4);
    superstructure.setTargetSourceSide(TargetSourceSide.MIDDLE);
    superstructure.setWantedState(WantedState.DEFAULT_STATE);

    autosBuilder = new AutosBuilder(superstructure);

    // Zero gyro while dissabled and sees tags :)
    driver
        .rightStick()
        .and(driver.leftStick())
        .onTrue(Commands.runOnce(() -> drivetrain.poseWithVisionPose()).ignoringDisable(true));

    if (Constants.babyControlMode) {
      configureBabyBindings();
    } else {
      configureNotBabyBindings();
    }
  }

  private void configureBabyBindings() {
    driver
        .rightTrigger()
        .onTrue(
            superstructure
                .configureButtonBinding(
                    WantedState.AUTO_DRIVE_TO_REEF,
                    WantedState.SCORE_ALGAE,
                    WantedState.INTAKE_ALGAE_FROM_REEF)
                .ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .leftTrigger()
        .onTrue(superstructure.setWantedStateCommand(WantedState.INTAKE).ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .leftBumper()
        .onTrue(Commands.runOnce(() -> superstructure.decideGamePieceScore()).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> superstructure.stopGamePieceScore()).ignoringDisable(true));

    driver
        .y()
        .and(driver.rightTrigger().negate().and(driver.rightBumper().negate()))
        .and(superstructure::doesntHaveAlgae)
        .onTrue(superstructure.setWantedStateCommand(WantedState.MANUAL_L4).ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .x()
        .and(driver.rightTrigger().negate().and(driver.rightBumper().negate()))
        .onTrue(superstructure.setWantedStateCommand(WantedState.MANUAL_L3).ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .a()
        .and(driver.rightTrigger().negate().and(driver.rightBumper().negate()))
        .onTrue(superstructure.setWantedStateCommand(WantedState.MANUAL_L2).ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .b()
        .onTrue(Commands.runOnce(() -> superstructure.toggleTargetType()).ignoringDisable(true));

    driver
        .povUp()
        .onTrue(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    // driver
    //     .povLeft()
    //
    // .onTrue(superstructure.setWantedStateCommand(WantedState.PRE_CLIMB).ignoringDisable(true));

    // driver
    //     .povRight()
    //     .onTrue(superstructure.setWantedStateCommand(WantedState.CLIMB).ignoringDisable(true));

    driver
        .povDown()
        .onTrue(superstructure.setWantedStateCommand(WantedState.MANUAL_L1).ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .povLeft()
        .onTrue(Commands.runOnce(() -> superstructure.advanceAlgae()).ignoringDisable(true));

    driver
        .povLeft()
        .and(
            driver
                .leftTrigger()
                .or(
                    driver
                        .rightBumper()
                        .or(driver.rightTrigger().and(() -> !superstructure.hasAlgae()))))
        .onTrue(Commands.runOnce(() -> superstructure.advanceCoral()).ignoringDisable(true));

    driver
        .start()
        .onTrue(
            drivetrain
                .runOnce(
                    () ->
                        drivetrain.resetPose(
                            new Pose2d(
                                drivetrain.getPose().getX(),
                                drivetrain.getPose().getY(),
                                new Rotation2d())))
                .ignoringDisable(true));

    driver
        .back()
        .onTrue(
            Commands.runOnce(() -> superstructure.toggleCompressMaxSpeed()).ignoringDisable(true));

    endgame
        .onTrue(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0.5)))
        .onFalse(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0)));

    alphaButton
        .and(bravoButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ab, ReefSide.left))
                .ignoringDisable(true));
    bravoButton
        .and(alphaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ab, ReefSide.right))
                .ignoringDisable(true));
    charlieButton
        .and(deltaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.cd, ReefSide.left))
                .ignoringDisable(true));
    deltaButton
        .and(charlieButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.cd, ReefSide.right))
                .ignoringDisable(true));
    echoButton
        .and(foxtrotButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ef, ReefSide.left))
                .ignoringDisable(true));
    foxtrotButton
        .and(echoButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ef, ReefSide.right))
                .ignoringDisable(true));
    golfButton
        .and(hotelButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.gh, ReefSide.left))
                .ignoringDisable(true));
    hotelButton
        .and(golfButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.gh, ReefSide.right))
                .ignoringDisable(true));
    indiaButton
        .and(julietButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ij, ReefSide.left))
                .ignoringDisable(true));
    julietButton
        .and(indiaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ij, ReefSide.right))
                .ignoringDisable(true));
    kiloButton
        .and(limaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.kl, ReefSide.left))
                .ignoringDisable(true));
    limaButton
        .and(kiloButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.kl, ReefSide.right))
                .ignoringDisable(true));
    l4Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L4)).ignoringDisable(true));
    l3Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L3)).ignoringDisable(true));
    l2Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L2)).ignoringDisable(true));
    l1Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L1)).ignoringDisable(true));
  }

  private void configureNotBabyBindings() {
    driver.rightStick().onTrue(Commands.runOnce(() -> superstructure.targetByRotation()));

    driver.leftStick().onTrue(Commands.runOnce(() -> superstructure.targetByDistance()));

    driver
        .rightStick()
        .and(driver.leftStick())
        .onTrue(Commands.runOnce(() -> superstructure.stopAutoTargeting()));

    driver
        .rightTrigger()
        .onTrue(
            Commands.runOnce(() -> superstructure.setTargetSideIfAble(ReefSide.right))
                .andThen(Commands.runOnce(() -> superstructure.decideNextReefTargetFace())))
        .and(driver.rightBumper().negate())
        .onTrue(
            superstructure
                .configureButtonBinding(
                    WantedState.AUTO_DRIVE_TO_REEF,
                    WantedState.SCORE_ALGAE,
                    WantedState.INTAKE_ALGAE_FROM_REEF)
                .ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> superstructure.setTargetSideIfAble(ReefSide.left))
                .andThen(Commands.runOnce(() -> superstructure.decideNextReefTargetFace())))
        .and(driver.rightTrigger().negate())
        .onTrue(
            superstructure
                .configureButtonBinding(
                    WantedState.AUTO_DRIVE_TO_REEF,
                    WantedState.DEFAULT_STATE,
                    WantedState.INTAKE_ALGAE_FROM_REEF)
                .ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .leftTrigger()
        .onTrue(superstructure.setWantedStateCommand(WantedState.INTAKE).ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .leftBumper()
        .onTrue(Commands.runOnce(() -> superstructure.decideGamePieceScore()).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> superstructure.stopGamePieceScore()).ignoringDisable(true));

    driver
        .y()
        .and(driver.rightTrigger().or(driver.rightBumper()))
        .onTrue(
            superstructure
                .setWantedStateCommand(WantedState.SCORE_L4)
                .alongWith(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L4)))
                .ignoringDisable(true));

    driver
        .x()
        .and(driver.rightTrigger().or(driver.rightBumper()))
        .onTrue(
            superstructure
                .setWantedStateCommand(WantedState.SCORE_L3)
                .alongWith(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L3)))
                .ignoringDisable(true));

    driver
        .a()
        .and(driver.rightTrigger().or(driver.rightBumper()))
        .onTrue(
            superstructure
                .setWantedStateCommand(WantedState.SCORE_L2)
                .alongWith(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L2)))
                .ignoringDisable(true));

    driver
        .y()
        .and(driver.rightTrigger().negate().and(driver.rightBumper().negate()))
        .onTrue(
            superstructure
                .setWantedStateCommand(WantedState.MANUAL_L4)
                .alongWith(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L4)))
                .ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .x()
        .and(driver.rightTrigger().negate().and(driver.rightBumper().negate()))
        .onTrue(
            superstructure
                .setWantedStateCommand(WantedState.MANUAL_L3)
                .alongWith(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L3)))
                .ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .a()
        .and(driver.rightTrigger().negate().and(driver.rightBumper().negate()))
        .onTrue(
            superstructure
                .setWantedStateCommand(WantedState.MANUAL_L2)
                .alongWith(Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L2)))
                .ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .b()
        .onTrue(Commands.runOnce(() -> superstructure.toggleTargetType()).ignoringDisable(true));

    driver
        .povUp()
        .onTrue(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .povLeft()
        .onTrue(superstructure.setWantedStateCommand(WantedState.PRE_CLIMB).ignoringDisable(true));

    driver
        .povRight()
        .onTrue(superstructure.setWantedStateCommand(WantedState.CLIMB).ignoringDisable(true));

    driver
        .povDown()
        .and(driver.povRight())
        .onTrue(superstructure.setWantedStateCommand(WantedState.MANUAL_L1).ignoringDisable(true))
        .onFalse(
            superstructure.setWantedStateCommand(WantedState.DEFAULT_STATE).ignoringDisable(true));

    driver
        .povLeft()
        .onTrue(Commands.runOnce(() -> superstructure.advanceAlgae()).ignoringDisable(true));

    driver
        .povLeft()
        .and(
            driver
                .leftTrigger()
                .or(
                    driver
                        .rightBumper()
                        .or(driver.rightTrigger().and(() -> !superstructure.hasAlgae()))))
        .onTrue(Commands.runOnce(() -> superstructure.advanceCoral()).ignoringDisable(true));

    driver
        .start()
        .onTrue(
            drivetrain
                .runOnce(
                    () ->
                        drivetrain.resetPose(
                            new Pose2d(
                                drivetrain.getPose().getX(),
                                drivetrain.getPose().getY(),
                                new Rotation2d())))
                .ignoringDisable(true));

    driver
        .back()
        .onTrue(
            Commands.runOnce(() -> superstructure.toggleCompressMaxSpeed()).ignoringDisable(true));

    endgame
        .onTrue(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0.5)))
        .onFalse(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0)));

    alphaButton
        .and(bravoButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ab, ReefSide.left))
                .ignoringDisable(true));
    bravoButton
        .and(alphaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ab, ReefSide.right))
                .ignoringDisable(true));
    charlieButton
        .and(deltaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.cd, ReefSide.left))
                .ignoringDisable(true));
    deltaButton
        .and(charlieButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.cd, ReefSide.right))
                .ignoringDisable(true));
    echoButton
        .and(foxtrotButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ef, ReefSide.left))
                .ignoringDisable(true));
    foxtrotButton
        .and(echoButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ef, ReefSide.right))
                .ignoringDisable(true));
    golfButton
        .and(hotelButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.gh, ReefSide.left))
                .ignoringDisable(true));
    hotelButton
        .and(golfButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.gh, ReefSide.right))
                .ignoringDisable(true));
    indiaButton
        .and(julietButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ij, ReefSide.left))
                .ignoringDisable(true));
    julietButton
        .and(indiaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.ij, ReefSide.right))
                .ignoringDisable(true));
    kiloButton
        .and(limaButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.kl, ReefSide.left))
                .ignoringDisable(true));
    limaButton
        .and(kiloButton.negate())
        .onTrue(
            Commands.runOnce(() -> superstructure.setTarget(ReefFaces.kl, ReefSide.right))
                .ignoringDisable(true));
    l4Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L4)).ignoringDisable(true));
    l3Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L3)).ignoringDisable(true));
    l2Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L2)).ignoringDisable(true));
    l1Button.onTrue(
        Commands.runOnce(() -> superstructure.setTargetLevel(ReefLevel.L1)).ignoringDisable(true));
  }

  public void setAlliance(boolean alliance) {
    superstructure.setAlliance(alliance);
  }

  public void requsetDefault() {
    superstructure.setWantedState(WantedState.DEFAULT_STATE);
  }

  public Command getAutonomousCommand() {
    return autosBuilder.getAuto();
  }

  public void autoPeriodic() {
    autosBuilder.periodic();
  }
}
