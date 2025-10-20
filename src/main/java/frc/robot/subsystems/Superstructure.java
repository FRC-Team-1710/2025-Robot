// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutomationLevel;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SimCoralAutomation;
import frc.robot.autos.AutosBuilder.Reef;
import frc.robot.autos.AutosBuilder.ReefHeight;
import frc.robot.autos.AutosBuilder.Source;
import frc.robot.autos.AutosBuilder.SourceDistance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.LEDs.LEDSubsystem;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.Claw.ClawStates;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.climber.Climber.ClimberStates;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.funnel.Funnel.FunnelState;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.superstructure.manipulator.Manipulator.ManipulatorStates;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.AutomationLevelChooser;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.SimCoral;
import frc.robot.utils.SimCoralAutomationChooser;
import frc.robot.utils.TunableController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drivetrain;
  private final Claw claw;
  private final Climber climber;
  private final Elevator elevator;
  private final Funnel funnel;
  private final Manipulator manipulator;
  private final Vision vision;

  @SuppressWarnings("unused") // Sorry Gavin
  private final LEDSubsystem ledSubsystem;

  private final TunableController driver;
  private final TunableController mech;

  private final APConstraints constraints =
      new APConstraints()
          .withAcceleration(Constants.currentMode == Mode.SIM ? 50 : 75)
          .withVelocity(Constants.currentMode == Mode.SIM ? Double.POSITIVE_INFINITY : 3)
          .withJerk(Constants.currentMode == Mode.SIM ? 0.1 : 0.02);
  private final APProfile profile =
      new APProfile(constraints)
          .withErrorXY(Inches.of(1))
          .withErrorTheta(Degrees.of(1.5))
          .withBeelineRadius(Inches.of(24));

  private Autopilot autopilot = new Autopilot(profile);

  private APTarget currentTarget = new APTarget(new Pose2d());

  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState;

  private AutoAlignType currentAutoDriveType = AutoAlignType.PP;
  private AlignTarget currentAlignTarget = AlignTarget.REEF;

  private ReefFaces targetFace = ReefFaces.ab;
  private ReefSide targetSide = ReefSide.left;
  private ReefLevel targetLevel = ReefLevel.L4;
  private TargetSourceSide targetSourceSide = TargetSourceSide.FAR;

  private TargetingMethod targetingMethod = TargetingMethod.ROTATION;

  private TargetType targetingType = TargetType.CORAL;

  private GamePiecePositions currentGamePiecePosition = GamePiecePositions.NONE;

  private AutomationLevel automationLevel = AutomationLevel.AUTO_DRIVE;
  private SimCoralAutomation simCoralAutomation = SimCoralAutomation.AUTO_SIM_CORAL;

  private final AutomationLevelChooser automationLevelChooser;
  private final SimCoralAutomationChooser simCoralAutomationChooser;

  private double driverOverideAllignment = 0.25;

  private final double metersToElevatorUp = 0.75;

  private boolean autoSourceIsLeft = false;

  private boolean scoreCoralFlag = false;

  private boolean manualScoreCoralBeingFlagged = false;

  private boolean hasScoredCoralSim = false;

  private boolean isRedAlliance = false;

  private boolean compressMaxSpeed = true;

  private boolean ppReady = false;

  private final double offsetX = Units.inchesToMeters(17.5);
  private final double offsetY = Units.inchesToMeters(7);

  private final Timer ejectTimer = new Timer();

  private LinearVelocity maxSpeed = TunerConstants.kSpeedAt12Volts;

  private AngularVelocity maxAngularRate = Constants.MaxAngularRate;

  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed.times(0.025))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.025))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PIDController movingRotation =
      new PIDController(
          Constants.currentMode == Mode.SIM ? 0.03 : 0.01,
          0,
          Constants.currentMode == Mode.SIM ? 0.00175 : 0);

  private Command currentPathFindingCommand = Commands.none();
  private PathConstraints pathfindingConstraints = PathConstraints.unlimitedConstraints(12);

  private Command ppWUp =
      FollowPathCommand.warmupCommand().andThen(PathfindingCommand.warmupCommand());

  // new PathConstraints(
  //     maxSpeed,
  //     MetersPerSecondPerSecond.of(50),
  //     maxAngularRate,
  //     RotationsPerSecondPerSecond.of(15));

  public Superstructure(
      Drive drivetrain,
      Claw claw,
      Climber climber,
      Elevator elevator,
      Funnel funnel,
      LEDSubsystem ledSubsystem,
      Manipulator manipulator,
      Vision vision,
      TunableController driver,
      TunableController mech) {
    this.drivetrain = drivetrain;
    this.claw = claw;
    this.climber = climber;
    this.elevator = elevator;
    this.funnel = funnel;
    this.ledSubsystem = ledSubsystem;
    this.manipulator = manipulator;
    this.vision = vision;
    this.driver = driver;
    this.mech = mech;
    this.automationLevelChooser = new AutomationLevelChooser();
    this.simCoralAutomationChooser = new SimCoralAutomationChooser();

    ppWUp.schedule();

    if (Constants.useSmartDashboard) {
      SmartDashboard.putBoolean("Superstructure/Sim/AdvanceGamePiece", false);
      SmartDashboard.putNumber("Acceleration", 0);
      SmartDashboard.putNumber("Jerk", 0);
      SmartDashboard.putNumber("Velocity", 0);
    }
  }

  public boolean driverRumble() {
    return elevator.isAtTarget()
        && elevator.getState() != ElevatorStates.INTAKE
        && isDrivetrainAtTarget()
        && manipulator.hasCoral();
  }

  public boolean isPathFindingFinishedAuto() {
    return switch (currentAutoDriveType) {
      case AP -> isDrivetrainAtTarget() && DriverStation.isAutonomous();
      case PP -> false; // Waits for the automatic switch to AutoPilot allign
    };
  }

  public void setAlliance(boolean redAlliance) {
    isRedAlliance = redAlliance;
  }

  @Override
  public void periodic() {
    if (Constants.useSmartDashboard) {
      autopilot =
          new Autopilot(
              profile.withConstraints(
                  constraints
                      .withAcceleration(SmartDashboard.getNumber("Acceleration", 0))
                      .withVelocity(SmartDashboard.getNumber("Velocity", 0))
                      .withJerk(SmartDashboard.getNumber("Jerk", 0))));
    }

    ppReady = (!ppWUp.isScheduled());

    if (Constants.currentMode == Mode.SIM && hasAlgae() && manipulator.detectsCoral()) {
      manipulator.advanceGamePiece();
    }

    automationLevel = automationLevelChooser.getAutomationLevel();
    simCoralAutomation = simCoralAutomationChooser.getAutomationLevel();

    currentGamePiecePosition =
        manipulator.hasCoral()
            ? GamePiecePositions.CORAL_SECURED_IN_MANIPULATOR
            : manipulator.detectsCoral()
                ? GamePiecePositions.CORAL_IN_MANIPULATOR
                : funnel.hasCoral()
                    ? GamePiecePositions.CORAL_IN_FUNNEL
                    : claw.hasAlgae() ? GamePiecePositions.ALGAE_IN_CLAW : GamePiecePositions.NONE;

    maxAngularRate = Constants.MaxAngularRate.times(claw.hasAlgae() ? 0.5 : 1);

    if (elevator.getPosition().in(Inches) < 15 || !compressMaxSpeed) {
      maxSpeed = TunerConstants.kSpeedAt12Volts;
      Logger.recordOutput("Superstructure/MaxSpeedCompression", 1);
    } else if (elevator.getPosition().in(Inches) > 45) {
      maxSpeed = TunerConstants.kSpeedAt12Volts.times(0.5);
      Logger.recordOutput("Superstructure/MaxSpeedCompression", 0.5);
    } else {
      maxSpeed =
          TunerConstants.kSpeedAt12Volts.times(1 - ((elevator.getPosition().in(Inches) - 15) / 60));
      Logger.recordOutput(
          "Superstructure/MaxSpeedCompression",
          1 - ((elevator.getPosition().in(Inches) - 15) / 60));
    }

    Logger.recordOutput(
        "AP/ErrorTrans",
        Units.metersToInches(
            drivetrain
                .getPose()
                .getTranslation()
                .getDistance(currentTarget.getReference().getTranslation())));
    Logger.recordOutput(
        "AP/ErrorTransM",
        drivetrain
            .getPose()
            .getTranslation()
            .getDistance(currentTarget.getReference().getTranslation()));
    Logger.recordOutput(
        "AP/ErrorRot",
        drivetrain
            .getPose()
            .getRotation()
            .minus(currentTarget.getReference().getRotation())
            .getDegrees());

    Logger.recordOutput("Superstructure/WantedState", wantedState);
    Logger.recordOutput("Superstructure/currentState", currentState);
    Logger.recordOutput("Superstructure/PreviousState", previousState);

    Logger.recordOutput("Superstructure/CurrentGamePiecePosition", currentGamePiecePosition);

    Logger.recordOutput(
        "Superstructure/TargetSourcePoseAuto", targetSourcePoseAuto(drivetrain.getPose()));

    Logger.recordOutput("Superstructure/TargetFace", targetFace);
    Logger.recordOutput("Superstructure/TargetSide", targetSide);
    Logger.recordOutput("Superstructure/TargetLevel", targetLevel);
    Logger.recordOutput("Superstructure/TargetSourceAutoIsLeft", autoSourceIsLeft);
    Logger.recordOutput("Superstructure/TargetSourceDistance", targetSourceSide);

    Logger.recordOutput("Superstructure/PPReady", ppReady);

    Logger.recordOutput("Superstructure/DriverRumble", driverRumble());

    driver.setRumble(RumbleType.kBothRumble, driverRumble() ? 1 : 0);

    if (!scoreCoralFlag) {
      ejectTimer.reset();
      if (ejectTimer.isRunning()) ejectTimer.stop();
      hasScoredCoralSim = false;
    }

    currentState = handStateTransitions();
    applyStates();
  }

  @AutoLogOutput(key = "Superstructure/CurrentState")
  private CurrentState handStateTransitions() {
    previousState = currentState;
    if (wantedState == WantedState.SCORE_AUTO) {
      wantedState =
          switch (targetLevel) {
            case L1 -> wantedState;
            case L2 -> WantedState.SCORE_L2;
            case L3 -> WantedState.SCORE_L3;
            case L4 -> WantedState.SCORE_L4;
          };
    }
    if (wantedState == WantedState.SCORE_ALGAE) {
      currentState = decideStateForAlgae();
    } else {
      switch (wantedState) {
        case ZERO:
          currentState = CurrentState.ZERO;
          break;
        case DEFAULT_STATE:
          if (currentPathFindingCommand.isScheduled()) {
            currentPathFindingCommand.cancel();
          }
          if (manipulator.hasCoral()) {
            if (DriverStation.isAutonomous()) {
              currentState = CurrentState.HOLDING_CORAL_AUTO;
            } else {
              currentState = CurrentState.HOLDING_CORAL_TELEOP;
            }
          } else if (claw.hasAlgae()) {
            currentState = CurrentState.HOLDING_ALGAE;
          } else {
            if (DriverStation.isAutonomous()) {
              currentState = CurrentState.NO_PIECE_AUTO;
            } else {
              currentState = CurrentState.NO_PIECE_TELEOP;
            }
          }
          break;
        case INTAKE_CORAL_FROM_STATION:
          currentState =
              DriverStation.isAutonomous()
                  ? CurrentState.INTAKE_CORAL_FROM_STATION_AUTO
                  : CurrentState.INTAKE_CORAL_FROM_STATION;
          break;
        case AUTO_DRIVE_TO_CORAL_STATION:
          if (automationLevel != AutomationLevel.NO_AUTO_DRIVE || DriverStation.isAutonomous()) {
            if (manipulator.hasCoral()) {
              currentState =
                  manipulator.hasCoral()
                      ? CurrentState.HOLDING_CORAL_TELEOP
                      : claw.hasAlgae() ? CurrentState.HOLDING_ALGAE : CurrentState.NO_PIECE_TELEOP;
            } else {
              currentState = CurrentState.AUTO_DRIVE_TO_CORAL_STATION;
            }
          } else {
            currentState =
                manipulator.hasCoral()
                    ? CurrentState.HOLDING_CORAL_TELEOP
                    : claw.hasAlgae() ? CurrentState.HOLDING_ALGAE : CurrentState.NO_PIECE_TELEOP;
          }
          break;
        case AUTO_DRIVE_TO_REEF:
          if (automationLevel != AutomationLevel.NO_AUTO_DRIVE || DriverStation.isAutonomous()) {
            currentState = CurrentState.AUTO_DRIVE_TO_REEF;
          } else {
            currentState =
                manipulator.hasCoral()
                    ? CurrentState.HOLDING_CORAL_TELEOP
                    : claw.hasAlgae() ? CurrentState.HOLDING_ALGAE : CurrentState.NO_PIECE_TELEOP;
          }
          break;
        case SCORE_L2:
          currentState =
              DriverStation.isAutonomous()
                  ? CurrentState.SCORE_AUTO_L2
                  : CurrentState.SCORE_TELEOP_L2;
          break;
        case SCORE_L3:
          currentState =
              DriverStation.isAutonomous()
                  ? CurrentState.SCORE_AUTO_L3
                  : CurrentState.SCORE_TELEOP_L3;
          break;
        case SCORE_L4:
          currentState =
              DriverStation.isAutonomous()
                  ? CurrentState.SCORE_AUTO_L4
                  : CurrentState.SCORE_TELEOP_L4;
          break;
        case MANUAL_L1:
          currentState = CurrentState.MANUAL_L1;
          break;
        case MANUAL_L2:
          currentState = CurrentState.MANUAL_L2;
          break;
        case MANUAL_L3:
          currentState = CurrentState.MANUAL_L3;
          break;
        case MANUAL_L4:
          currentState = CurrentState.MANUAL_L4;
          break;
        case INTAKE:
          currentState =
              targetingType == TargetType.CORAL
                  ? CurrentState.INTAKE_CORAL_FROM_STATION
                  : CurrentState.INTAKE_ALGAE_FROM_REEF;
          break;
        case INTAKE_ALGAE_FROM_REEF:
          currentState = CurrentState.INTAKE_ALGAE_FROM_REEF;
          break;
        case INTAKE_ALGAE_FROM_GROUND:
          currentState = CurrentState.INTAKE_ALGAE_FROM_GROUND;
          break;
        case MOVE_ALGAE_TO_NET_POSITION:
          currentState = CurrentState.MOVE_ALGAE_TO_NET_POSITION;
          break;
        case MOVE_ALGAE_TO_PROCESSOR_POSITION:
          currentState = CurrentState.MOVE_ALGAE_TO_PROCESSOR_POSITION;
          break;
        case SCORE_ALGAE_IN_NET:
          currentState = CurrentState.SCORE_ALGAE_IN_NET;
          break;
        case SCORE_ALGAE_IN_PROCESSOR:
          currentState = CurrentState.SCORE_ALGAE_IN_PROCESSOR;
          break;
        case PRE_CLIMB:
          currentState = CurrentState.PRE_CLIMB;
          break;
        case CLIMB:
          currentState = CurrentState.CLIMB;
          break;
        case CLIMB_MANUAL:
          currentState = CurrentState.CLIMB_MANUAL;
          break;
        default:
          currentState = CurrentState.STOPPED;
          break;
      }
    }
    // Ensure you can't move anything after deploying climber
    if (previousState == CurrentState.PRE_CLIMB && currentState != CurrentState.CLIMB) {
      currentState = CurrentState.PRE_CLIMB;
    } else if (previousState == CurrentState.CLIMB && currentState != CurrentState.CLIMB_MANUAL) {
      currentState = CurrentState.CLIMB;
    } else if (previousState == CurrentState.CLIMB_MANUAL) {
      currentState = CurrentState.CLIMB_MANUAL;
    }
    return currentState;
  }

  private void applyStates() {
    switch (currentState) {
      case AUTO_DRIVE_TO_CORAL_STATION, AUTO_DRIVE_TO_REEF:
        driverOverideAllignment = 0;
        break;
      default:
        driverOverideAllignment = 0.25;
        break;
    }
    switch (currentState) {
      case ZERO:
        zero();
        break;
      case INTAKE_CORAL_FROM_STATION:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(true);
        }
        intakeCoralFromStation();
        break;
      case INTAKE_CORAL_FROM_STATION_AUTO:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(true);
        }
        intakeCoralFromStationAuto();
        break;
      case AUTO_DRIVE_TO_CORAL_STATION:
        if (DriverStation.isAutonomous()) {
          intakeCoralFromStationAuto();
        } else {
          intakeCoralFromStation();
        }
        break;
      case NO_PIECE_TELEOP:
        noPiece();
        break;
      case NO_PIECE_AUTO:
        noPiece();
        break;
      case HOLDING_CORAL_AUTO:
        holdingCoral();
        break;
      case HOLDING_CORAL_TELEOP:
        holdingCoral();
        break;
      case HOLDING_ALGAE:
        holdingAlgae();
        break;
      case AUTO_DRIVE_TO_REEF:
        autoDriveToReef();
        break;
      case SCORE_TELEOP_L2:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(false);
        }
        scoreL2Teleop();
        break;
      case SCORE_TELEOP_L3:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(false);
        }
        scoreL3Teleop();
        break;
      case SCORE_TELEOP_L4:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(false);
        }
        scoreL4Teleop();
        break;
      case SCORE_AUTO_L2:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(false);
        }
        scoreL2Auto();
        break;
      case SCORE_AUTO_L3:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(false);
        }
        scoreL3Auto();
        break;
      case SCORE_AUTO_L4:
        if (Constants.currentMode == Mode.SIM) {
          advanceCoral(false);
        }
        scoreL4Auto();
        break;
      case MANUAL_L4:
        manualL4();
        break;
      case MANUAL_L3:
        manualL3();
        break;
      case MANUAL_L2:
        manualL2();
        break;
      case MANUAL_L1:
        manualL1();
        break;
      case INTAKE_ALGAE_FROM_REEF:
        intakeAlgaeFromReef();
        break;
      case INTAKE_ALGAE_FROM_GROUND:
        intakeAlgaeFromGround();
        break;
      case SCORE_ALGAE_IN_NET:
        if (Constants.currentMode == Mode.SIM && claw.hasAlgae()) {
          advanceAlgae();
        }
        scoreAlgaeNet();
        break;
      case SCORE_ALGAE_IN_PROCESSOR:
        if (Constants.currentMode == Mode.SIM && claw.hasAlgae()) {
          advanceAlgae();
        }
        scoreAlgaeProcessor();
        break;
      case MOVE_ALGAE_TO_NET_POSITION:
        moveAlgaeToNetPosition();
        break;
      case MOVE_ALGAE_TO_PROCESSOR_POSITION:
        moveAlgaeToProcessorPosition();
        break;
      case PRE_CLIMB:
        preClimb();
        break;
      case CLIMB:
        climb();
        break;
      case CLIMB_MANUAL:
        climbManual();
        break;
      case STOPPED:
        stopped();
        break;
    }
  }

  /** Advances coral in the sim */
  private void advanceCoral(boolean intaking) {
    if ((simCoralAutomation == SimCoralAutomation.AUTO_SIM_CORAL
            && ((scoreCoralFlag && !hasScoredCoralSim) || (intaking)))
        || (manualScoreCoralBeingFlagged && !hasScoredCoralSim)) {
      if (!intaking) {
        hasScoredCoralSim = true;
      }
      manipulator.advanceGamePiece();
    }
  }

  /** Zeros Wrist and Elevator */
  private void zero() {
    if (claw.isDoneZeroing() && previousState == CurrentState.ZERO) {
      claw.setState(ClawStates.IDLE);
    } else {
      claw.setState(ClawStates.ZERO);
    }

    if (elevator.isDoneZeroing() && previousState == CurrentState.ZERO) {
      elevator.setState(ElevatorStates.INTAKE);
    } else {
      elevator.setState(ElevatorStates.ZERO);
    }

    if (claw.getState() == ClawStates.IDLE && elevator.getState() == ElevatorStates.INTAKE) {
      setWantedState(WantedState.DEFAULT_STATE);
    }
  }

  private void intakeCoralFromStation() {
    scoreCoralFlag = false;
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    manipulator.setState(ManipulatorStates.INTAKE);
    funnel.setState(manipulator.detectsCoral() ? FunnelState.INTAKE_SLOW : FunnelState.INTAKE);
    applyDrive(targetSourcePoseAuto(drivetrain.getPose()).getRotation());
    if (manipulator.hasCoral()) {
      setWantedState(WantedState.DEFAULT_STATE);
    }
  }

  private void intakeCoralFromStationAuto() {
    scoreCoralFlag = false;
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    manipulator.setState(ManipulatorStates.INTAKE);
    funnel.setState(manipulator.detectsCoral() ? FunnelState.INTAKE_SLOW : FunnelState.INTAKE);
    currentAlignTarget = AlignTarget.SOURCE;
    applyDrive(targetSourcePoseAuto(drivetrain.getPose()));
    if (manipulator.hasCoral()) {
      setWantedState(WantedState.DEFAULT_STATE);
    }
  }

  private void noPiece() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    applyDrive();
  }

  private void holdingCoral() {
    scoreCoralFlag = false;
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    if (manipulator.hasCoral()) {
      funnel.setState(FunnelState.OFF);
      manipulator.setState(ManipulatorStates.OFF);
    } else {
      funnel.setState(FunnelState.INTAKE_SLOW);
      manipulator.setState(ManipulatorStates.INTAKE);
    }
    applyDrive();
  }

  private void holdingAlgae() {
    claw.setState(ClawStates.HOLD);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    applyDrive();
  }

  private void autoDriveToReef() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);

    currentAlignTarget = AlignTarget.REEF;
    applyDrive(
        (!manipulator.hasCoral() || targetFace.inverted)
            ? getBeforeReadyToGrabAlgaePose()
            : getTargetPose());

    if (isDrivetrainNearTarget()) {
      setWantedState(
          manipulator.hasCoral()
              ? targetLevel == ReefLevel.L4
                  ? WantedState.SCORE_L4
                  : targetLevel == ReefLevel.L3
                      ? WantedState.SCORE_L3
                      : targetLevel == ReefLevel.L2 ? WantedState.SCORE_L2 : WantedState.MANUAL_L1
              : WantedState.INTAKE_ALGAE_FROM_REEF);
    }
  }

  private void scoreL2Teleop() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L2);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    currentAlignTarget = AlignTarget.REEF;
    applyDrive(getTargetPose());
    if (!manipulator.detectsCoral()) {
      if (Constants.currentMode == Mode.SIM) {
        SimCoral.addPose(targetFace, targetSide, targetLevel);
      }
    }
  }

  private void scoreL3Teleop() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L3);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    currentAlignTarget = AlignTarget.REEF;
    applyDrive(getTargetPose());
    if (!manipulator.detectsCoral()) {
      if (Constants.currentMode == Mode.SIM) {
        SimCoral.addPose(targetFace, targetSide, targetLevel);
      }
    }
  }

  private void scoreL4Teleop() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L4);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    currentAlignTarget = AlignTarget.REEF;
    applyDrive(getTargetPose());
    if (!manipulator.detectsCoral()) {
      if (Constants.currentMode == Mode.SIM) {
        SimCoral.addPose(targetFace, targetSide, targetLevel);
      }
    }
  }

  private void scoreL2Auto() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L2);
    funnel.setState(FunnelState.OFF);
    scoreCoralFlag = ((isDrivetrainAtTarget() && elevator.isAtTarget()) || scoreCoralFlag);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    currentAlignTarget = AlignTarget.REEF;
    applyDrive(getTargetPose());
    if (!manipulator.detectsCoral()) {
      ejectTimer.start();
      if (Constants.currentMode == Mode.SIM) {
        SimCoral.addPose(targetFace, targetSide, targetLevel);
      }
      if (ejectTimer.hasElapsed(0.5)) {
        setWantedState(WantedState.DEFAULT_STATE);
      }
    }
  }

  private void scoreL3Auto() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L3);
    funnel.setState(FunnelState.OFF);
    scoreCoralFlag = ((isDrivetrainAtTarget() && elevator.isAtTarget()) || scoreCoralFlag);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    currentAlignTarget = AlignTarget.REEF;
    applyDrive(getTargetPose());
    if (!manipulator.detectsCoral()) {
      ejectTimer.start();
      if (Constants.currentMode == Mode.SIM) {
        SimCoral.addPose(targetFace, targetSide, targetLevel);
      }
      if (ejectTimer.hasElapsed(0.5)) {
        setWantedState(WantedState.DEFAULT_STATE);
      }
    }
  }

  private void scoreL4Auto() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L4);
    funnel.setState(FunnelState.OFF);
    scoreCoralFlag = ((isDrivetrainAtTarget() && elevator.isAtTarget()) || scoreCoralFlag);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    if (ejectTimer.hasElapsed(0.5)) {
      currentAlignTarget = AlignTarget.AP;
      applyDrive(getTargetPose().plus(new Transform2d(-0.5, 0, Rotation2d.kZero)));
    } else {
      currentAlignTarget = AlignTarget.REEF;
      applyDrive(getTargetPose());
    }
    if (!manipulator.detectsCoral()) {
      ejectTimer.start();
      if (Constants.currentMode == Mode.SIM) {
        SimCoral.addPose(targetFace, targetSide, targetLevel);
      }
      if (ejectTimer.hasElapsed(1)) {
        setWantedState(WantedState.DEFAULT_STATE);
      }
    }
  }

  private void manualL1() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    funnel.setState(FunnelState.OFF);
    if (elevator.getState() != ElevatorStates.L1 && elevator.getState() != ElevatorStates.L2) {
      elevator.setState(ElevatorStates.L1);
      manipulator.setState(ManipulatorStates.OFF);
    } else if (elevator.getState() == ElevatorStates.L1 && elevator.isAtTarget()) {
      elevator.setState(ElevatorStates.L2);
      manipulator.setState(ManipulatorStates.OUTTAKE);
    } else if (elevator.getState() == ElevatorStates.L2 && elevator.isAtTarget()) {
      elevator.setState(ElevatorStates.INTAKE);
      manipulator.setState(ManipulatorStates.OFF);
      if (Constants.currentMode == Mode.SIM) {
        manipulator.advanceGamePiece();
      }
      setWantedState(WantedState.DEFAULT_STATE);
    }
    applyDrive();
  }

  private void manualL2() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L2);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    if (Constants.currentMode == Mode.SIM) {
      manipulator.advanceGamePiece();
    }
    applyDrive();
  }

  private void manualL3() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L3);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    if (Constants.currentMode == Mode.SIM) {
      manipulator.advanceGamePiece();
    }
    applyDrive();
  }

  private void manualL4() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L4);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
    if (Constants.currentMode == Mode.SIM) {
      manipulator.advanceGamePiece();
    }
    applyDrive();
  }

  private void intakeAlgaeFromReef() {
    climber.setState(ClimberStates.STOWED);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    if (claw.getState() != ClawStates.GRAB
        || !claw.isAtTarget()
        || elevator.getState()
            != (targetFace.isHighAlgae() ? ElevatorStates.ALGAE_HIGH : ElevatorStates.ALGAE_LOW)
        || !elevator.isAtTarget()) {
      applyDrive(getBeforeReadyToGrabAlgaePose());
      if (drivetrain
              .getPose()
              .getTranslation()
              .getDistance(getBeforeReadyToGrabAlgaePose().getTranslation())
          < Units.inchesToMeters(2.5)) {
        elevator.setState(
            targetFace.isHighAlgae() ? ElevatorStates.ALGAE_HIGH : ElevatorStates.ALGAE_LOW);
        claw.setState(ClawStates.GRAB);
      }
    } else if (!claw.hasAlgae()) {
      currentAlignTarget = AlignTarget.REEF;
      applyDrive(getReadyToGrabAlgaePose());
    } else if (claw.hasAlgae()) {
      currentAlignTarget = AlignTarget.AP;
      applyDrive(getBeforeReadyToGrabAlgaePose());
      if (drivetrain
              .getPose()
              .getTranslation()
              .getDistance(getBeforeReadyToGrabAlgaePose().getTranslation())
          < Units.inchesToMeters(2.5)) {
        setWantedState(WantedState.DEFAULT_STATE);
      }
    }
  }

  private void intakeAlgaeFromGround() {
    claw.setState(ClawStates.GRAB);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    if (vision.algaeIsVisible()) {
      applyDrive(vision.getAlgaeYaw() * 0.4);
    } else {
      applyDrive();
    }
    if (claw.hasAlgae()) {
      setWantedState(WantedState.DEFAULT_STATE);
    }
  }

  private void scoreAlgaeNet() {
    claw.setState(ClawStates.SCORE_NET);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L4);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    double angle1 =
        Math.abs(
            drivetrain
                .getPose()
                .getRotation()
                .minus(Rotation2d.fromDegrees(isRedAlliance ? 180 + 45 : 45))
                .minus(Rotation2d.fromDegrees(-driver.customRight().getX() * 12.5))
                .getDegrees());
    double angle2 =
        Math.abs(
            drivetrain
                .getPose()
                .getRotation()
                .minus(Rotation2d.fromDegrees(isRedAlliance ? 180 - 45 : -45))
                .minus(Rotation2d.fromDegrees(-driver.customRight().getX() * 12.5))
                .getDegrees());
    applyDrive(
        Rotation2d.fromDegrees(
            isRedAlliance ? 180 + (angle1 <= angle2 ? 45 : -45) : (angle1 <= angle2 ? 45 : -45)));
  }

  private void scoreAlgaeProcessor() {
    claw.setState(ClawStates.SCORE_PROCESSOR);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    applyDrive(getProcessorRotation());
  }

  private void moveAlgaeToNetPosition() {
    claw.setState(ClawStates.NET);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.L4);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    double angle1 =
        Math.abs(
            drivetrain
                .getPose()
                .getRotation()
                .minus(Rotation2d.fromDegrees(isRedAlliance ? 180 + 45 : 45))
                .minus(Rotation2d.fromDegrees(-driver.customRight().getX() * 12.5))
                .getDegrees());
    double angle2 =
        Math.abs(
            drivetrain
                .getPose()
                .getRotation()
                .minus(Rotation2d.fromDegrees(isRedAlliance ? 180 - 45 : -45))
                .minus(Rotation2d.fromDegrees(-driver.customRight().getX() * 12.5))
                .getDegrees());
    applyDrive(
        Rotation2d.fromDegrees(
            isRedAlliance ? 180 + (angle1 <= angle2 ? 45 : -45) : (angle1 <= angle2 ? 45 : -45)));
  }

  private void moveAlgaeToProcessorPosition() {
    claw.setState(ClawStates.PROCESSOR);
    climber.setState(ClimberStates.STOWED);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.OFF);
    manipulator.setState(ManipulatorStates.OFF);
    applyDrive(getProcessorRotation());
  }

  private void preClimb() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.OUT);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.CLIMB);
    manipulator.setState(ManipulatorStates.OFF);
    applyDrive();
  }

  private void climb() {
    claw.setState(ClawStates.IDLE);
    climber.setState(ClimberStates.CLIMBED);
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.CLIMB);
    manipulator.setState(ManipulatorStates.OFF);
    applyDrive();
    if (climber.hasClimbed()) {
      setWantedState(WantedState.CLIMB_MANUAL);
    }
  }

  private void climbManual() {
    claw.setState(ClawStates.IDLE);
    climber.setState(mech.getRightTriggerAxis() - mech.getLeftTriggerAxis());
    elevator.setState(ElevatorStates.INTAKE);
    funnel.setState(FunnelState.CLIMB);
    manipulator.setState(ManipulatorStates.OFF);
    applyDrive();
  }

  private void stopped() {
    claw.setState(ClawStates.STOP);
    climber.setState(0);
    elevator.setState(ElevatorStates.STOP);
    funnel.setState(FunnelState.STOP);
    manipulator.setState(ManipulatorStates.OFF);
  }

  /** Uses AP and PP to snap to specified pose */
  private void applyDrive(Pose2d pose) {
    Logger.recordOutput(
        "Superstructure/DistanceToRequestedPose",
        Math.abs(drivetrain.getPose().getTranslation().getDistance(pose.getTranslation())));
    if (ppReady
        && currentAlignTarget != AlignTarget.AP
        && ((currentAlignTarget == AlignTarget.REEF
                && isRobotOnWrongHalfOfReefFace(getTargetPose())
                && Math.abs(
                        drivetrain
                            .getPose()
                            .getTranslation()
                            .getDistance(getTargetPose().getTranslation()))
                    > 0.5)
            || (currentAlignTarget == AlignTarget.SOURCE
                && (targetFace == ReefFaces.ef
                    || targetFace == ReefFaces.gh
                    || targetFace == ReefFaces.ij)
                && Math.abs(
                        drivetrain.getPose().getTranslation().getDistance(pose.getTranslation()))
                    > 3.5))) {
      currentAutoDriveType = AutoAlignType.PP;
    } else {
      currentAutoDriveType = AutoAlignType.AP;
    }
    switch (currentAutoDriveType) {
      case AP:
        if (currentPathFindingCommand.isScheduled()) {
          currentPathFindingCommand.cancel();
        }
        Pose2d newPose =
            new Pose2d(pose.getTranslation(), Rotation2d.kZero)
                .plus(
                    new Transform2d(
                        (-driver.customLeft().getY()
                            * driverOverideAllignment
                            * (isRedAlliance ? -1 : 1)),
                        (-driver.customLeft().getX()
                            * driverOverideAllignment
                            * (isRedAlliance ? -1 : 1)),
                        Rotation2d.fromDegrees(-driver.customRight().getX() * 12.5)
                            .plus(pose.getRotation())));
        if (currentState == CurrentState.AUTO_DRIVE_TO_REEF && isRobotOnWrongHalfOfReefFace(pose)) {
          currentTarget =
              new APTarget(newPose)
                  .withEntryAngle(
                      pose.getRotation()
                          .plus(
                              Rotation2d.fromDegrees(
                                  isRobotOnLeftHalfOfReefFace(pose) ? -90 : 90)));
        } else {
          currentTarget = new APTarget(newPose).withoutEntryAngle();
        }

        var output =
            autopilot.calculate(drivetrain.getPose(), drivetrain.getChassisSpeeds(), currentTarget);

        double vx = output.vx().in(MetersPerSecond);
        double vy = output.vy().in(MetersPerSecond);

        double mx = Math.max(Math.abs(vx), Math.abs(vy));

        // clamp both while keeping direction
        if (mx > 1) {
          vx *= mx;
          vy *= mx;
        }

        Logger.recordOutput("AP/AppliedX%", clamp(vx));
        Logger.recordOutput("AP/AppliedY%", clamp(vx));

        applyDrive(
            clamp(vx * (isRedAlliance ? -1 : 1)),
            clamp(vy * (isRedAlliance ? -1 : 1)),
            output.targetAngle());
        break;
      case PP:
        if (!currentPathFindingCommand.isScheduled()) {
          // when PP is in a wall it tweaks so AP take over for a little
          // currentAlignTarget = AlignTarget.AP;
          // applyDrive(pose);

          // end at 100 to make sure it goes as fast as possible
          currentPathFindingCommand = AutoBuilder.pathfindToPose(pose, pathfindingConstraints, 100);
          currentPathFindingCommand.schedule();
        }
    }
  }

  public enum AutoAlignType {
    PP(),
    AP()
  }

  /**
   * Uses custom x and y velocities with rotation snap and the option to automatically add user
   * input
   *
   * <p>DOESN'T APPLY DRIVER CORRECTION!!!
   *
   * <p>ALSO DOESN'T CLAMP TRANSLATION!!!
   */
  private void applyDrive(double x, double y, Rotation2d rotationSnap) {
    drivetrain
        .applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(maxSpeed.times(x))
                    .withVelocityY(maxSpeed.times(y))
                    .withRotationalRate(
                        maxAngularRate.times(
                            clamp(
                                movingRotation.calculate(
                                    drivetrain
                                        .getPose()
                                        .getRotation()
                                        .minus(rotationSnap)
                                        .getDegrees(),
                                    0)))))
        .schedule();
  }

  /**
   * @param rotation uses this for rotation + driver
   */
  private void applyDrive(double rotation) {
    drivetrain
        .applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(maxSpeed.times(-driver.customLeft().getY()))
                    .withVelocityY(maxSpeed.times(-driver.customLeft().getX()))
                    .withRotationalRate(
                        maxAngularRate.times(
                            clamp(rotation)
                                - (driver.customRight().getX() * driverOverideAllignment))))
        .schedule();
  }

  /** Uses normal driver controlls */
  private void applyDrive() {
    drivetrain
        .applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(maxSpeed.times(-driver.customLeft().getY()))
                    .withVelocityY(maxSpeed.times(-driver.customLeft().getX()))
                    .withRotationalRate(maxAngularRate.times(-driver.customRight().getX())))
        .schedule();
  }

  /** Uses normal driver controlls with a rotation snap */
  private void applyDrive(Rotation2d rotationSnap) {
    drivetrain
        .applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(maxSpeed.times(-driver.customLeft().getY()))
                    .withVelocityY(maxSpeed.times(-driver.customLeft().getX()))
                    .withRotationalRate(
                        maxAngularRate.times(
                            clamp(
                                movingRotation.calculate(
                                    drivetrain
                                        .getPose()
                                        .getRotation()
                                        .minus(rotationSnap)
                                        .minus(
                                            Rotation2d.fromDegrees(
                                                -driver.customRight().getX() * 12.5))
                                        .getDegrees(),
                                    0)))))
        // - (driver.customRight().getX() * driverOverideAllignment)))))
        .schedule();
  }

  private double clamp(double amount) {
    return Math.copySign(Math.abs(amount) > 1 ? 1 : amount, amount);
  }

  public boolean isDrivetrainAtTarget() {
    return autopilot.atTarget(drivetrain.getPose(), currentTarget);
  }

  public boolean isRobotOnWrongHalfOfReefFace(Pose2d pose) {
    Translation2d relativeTranslation =
        drivetrain.getPose().getTranslation().minus(pose.getTranslation());
    Translation2d referenceForwardVector =
        new Translation2d(pose.getRotation().getCos(), pose.getRotation().getSin());
    double dotProduct =
        relativeTranslation.getX() * referenceForwardVector.getX()
            + relativeTranslation.getY() * referenceForwardVector.getY();

    return dotProduct > 0;
  }

  public boolean isRobotOnLeftHalfOfReefFace(Pose2d pose) {
    Translation2d relativeTranslation =
        drivetrain.getPose().getTranslation().minus(pose.getTranslation());
    Rotation2d rightRotation = pose.getRotation().plus(Rotation2d.fromDegrees(-90.0));
    Translation2d referenceRightVector =
        new Translation2d(rightRotation.getCos(), rightRotation.getSin());
    double dotProduct =
        relativeTranslation.getX() * referenceRightVector.getX()
            + relativeTranslation.getY() * referenceRightVector.getY();

    return dotProduct < 0;
  }

  private Rotation2d getProcessorRotation() {
    return Rotation2d.fromDegrees(
        isRedAlliance ? onOtherHalfOfField() ? 270 : 90 : onOtherHalfOfField() ? 90 : 270);
  }

  private Pose2d targetSourcePoseAuto(Pose2d pose) {
    if (DriverStation.isAutonomous()) {
      return new Pose2d(
              FieldConstants.aprilTags
                  .getTagPose(autoSourceIsLeft ? isRedAlliance ? 1 : 13 : isRedAlliance ? 2 : 12)
                  .get()
                  .getTranslation()
                  .toTranslation2d(),
              Rotation2d.fromDegrees(
                  autoSourceIsLeft ? isRedAlliance ? 126 : 306 : isRedAlliance ? 234 : 54))
          .plus(
              new Transform2d(
                  0.5,
                  targetSourceSide == TargetSourceSide.FAR
                      ? 0.6
                      : targetSourceSide == TargetSourceSide.MIDDLE ? 0 : -0.6,
                  new Rotation2d()));
    }

    return new Pose2d(
            FieldConstants.aprilTags
                .getTagPose(
                    pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2
                        ? isRedAlliance ? 1 : 13
                        : isRedAlliance ? 2 : 12)
                .get()
                .getTranslation()
                .toTranslation2d(),
            Rotation2d.fromDegrees(
                pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2
                    ? isRedAlliance ? 234 : 306
                    : isRedAlliance ? 126 : 54))
        .plus(
            new Transform2d(
                0.5,
                targetSourceSide == TargetSourceSide.FAR
                    ? pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2 ? 0.6 : -0.6
                    : targetSourceSide == TargetSourceSide.MIDDLE
                        ? 0
                        : pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2 ? -0.6 : 0.6,
                new Rotation2d()));
  }

  @AutoLogOutput(key = "Superstructure/TargetPose")
  private Pose2d getTargetPose() {
    return new Pose2d(
            FieldConstants.aprilTags
                .getTagPose(getTagForFace())
                .get()
                .getTranslation()
                .toTranslation2d(),
            FieldConstants.aprilTags.getTagPose(getTagForFace()).get().getRotation().toRotation2d())
        .plus(
            new Transform2d(
                offsetX, isTargetSideLeft() ? -offsetY : offsetY, new Rotation2d(Math.PI)));
  }

  private boolean isDrivetrainNearTarget() {
    return Math.abs(
            new Pose2d(
                    FieldConstants.aprilTags
                        .getTagPose(getTagForFace())
                        .get()
                        .getTranslation()
                        .toTranslation2d(),
                    FieldConstants.aprilTags
                        .getTagPose(getTagForFace())
                        .get()
                        .getRotation()
                        .toRotation2d())
                .plus(
                    new Transform2d(
                        offsetX, isTargetSideLeft() ? -offsetY : offsetY, new Rotation2d(Math.PI)))
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()))
        < metersToElevatorUp;
  }

  private Pose2d getFacePose(ReefFaces face) {
    return new Pose2d(
            FieldConstants.aprilTags
                .getTagPose(getTagForFace(face))
                .get()
                .getTranslation()
                .toTranslation2d(),
            FieldConstants.aprilTags
                .getTagPose(getTagForFace(face))
                .get()
                .getRotation()
                .toRotation2d())
        .plus(new Transform2d(offsetX, 0, new Rotation2d(Math.PI)));
  }

  private Pose2d getBeforeReadyToGrabAlgaePose() {
    return new Pose2d(
            FieldConstants.aprilTags
                .getTagPose(getTagForFace())
                .get()
                .getTranslation()
                .toTranslation2d(),
            FieldConstants.aprilTags.getTagPose(getTagForFace()).get().getRotation().toRotation2d())
        .plus(new Transform2d(Units.inchesToMeters(32), 0, new Rotation2d(Math.PI)));
  }

  private Pose2d getReadyToGrabAlgaePose() {
    return new Pose2d(
            FieldConstants.aprilTags
                .getTagPose(getTagForFace())
                .get()
                .getTranslation()
                .toTranslation2d(),
            FieldConstants.aprilTags.getTagPose(getTagForFace()).get().getRotation().toRotation2d())
        .plus(new Transform2d(Units.inchesToMeters(23), 0, new Rotation2d(Math.PI)));
  }

  public void toggleCompressMaxSpeed() {
    compressMaxSpeed = !compressMaxSpeed;
  }

  public boolean isIntakingCoralSim() {
    return currentState == CurrentState.INTAKE_CORAL_FROM_STATION;
  }

  public void advanceGamePiece() {
    switch (currentState) {
      case INTAKE_CORAL_FROM_STATION:
        advanceCoral();
        break;
      case INTAKE_ALGAE_FROM_REEF, INTAKE_ALGAE_FROM_GROUND:
        advanceCoral();
        break;
      default:
        break;
    }
  }

  public void advanceCoral() {
    manipulator.advanceGamePiece();
  }

  public void advanceAlgae() {
    if (currentState == CurrentState.HOLDING_ALGAE
        || currentState == CurrentState.INTAKE_ALGAE_FROM_GROUND
        || currentState == CurrentState.INTAKE_ALGAE_FROM_REEF
        || currentState == CurrentState.MOVE_ALGAE_TO_NET_POSITION
        || currentState == CurrentState.MOVE_ALGAE_TO_PROCESSOR_POSITION
        || currentState == CurrentState.SCORE_ALGAE_IN_NET
        || currentState == CurrentState.SCORE_ALGAE_IN_PROCESSOR) {
      claw.advanceGamePiece();
    }
  }

  public void beginSimAuto() {
    while (!manipulator.hasCoral()) {
      manipulator.advanceGamePiece();
    }
  }

  public boolean hasAlgae() {
    return claw.hasAlgae();
  }

  public boolean doesntHaveAlgae() {
    return !claw.hasAlgae();
  }

  public WantedState getWantedState() {
    return this.wantedState;
  }

  public void targetByRotation() {
    this.targetingMethod = TargetingMethod.ROTATION;
  }

  public void targetByDistance() {
    this.targetingMethod = TargetingMethod.DISTANCE;
  }

  public void stopAutoTargeting() {
    this.targetingMethod = TargetingMethod.NO_AUTO_TARGET;
  }

  public void decideNextReefTargetFace() {
    ReefFaces newFace = ReefFaces.ab;
    switch (targetingMethod) {
      case NO_AUTO_TARGET:
        return;
      case ROTATION:
        double closestRotation =
            Math.abs(
                getFacePose(ReefFaces.ab)
                    .getRotation()
                    .minus(drivetrain.getPose().getRotation())
                    .getDegrees());
        if (closestRotation
            > Math.abs(
                getFacePose(ReefFaces.cd)
                    .getRotation()
                    .minus(drivetrain.getPose().getRotation())
                    .getDegrees())) {
          closestRotation =
              Math.abs(
                  getFacePose(ReefFaces.cd)
                      .getRotation()
                      .minus(drivetrain.getPose().getRotation())
                      .getDegrees());
          newFace = ReefFaces.cd;
        }
        if (closestRotation
            > Math.abs(
                getFacePose(ReefFaces.ef)
                    .getRotation()
                    .minus(drivetrain.getPose().getRotation())
                    .getDegrees())) {
          closestRotation =
              Math.abs(
                  getFacePose(ReefFaces.ef)
                      .getRotation()
                      .minus(drivetrain.getPose().getRotation())
                      .getDegrees());
          newFace = ReefFaces.ef;
        }
        if (closestRotation
            > Math.abs(
                getFacePose(ReefFaces.gh)
                    .getRotation()
                    .minus(drivetrain.getPose().getRotation())
                    .getDegrees())) {
          closestRotation =
              Math.abs(
                  getFacePose(ReefFaces.gh)
                      .getRotation()
                      .minus(drivetrain.getPose().getRotation())
                      .getDegrees());
          newFace = ReefFaces.gh;
        }
        if (closestRotation
            > Math.abs(
                getFacePose(ReefFaces.ij)
                    .getRotation()
                    .minus(drivetrain.getPose().getRotation())
                    .getDegrees())) {
          closestRotation =
              Math.abs(
                  getFacePose(ReefFaces.ij)
                      .getRotation()
                      .minus(drivetrain.getPose().getRotation())
                      .getDegrees());
          newFace = ReefFaces.ij;
        }
        if (closestRotation
            > Math.abs(
                getFacePose(ReefFaces.kl)
                    .getRotation()
                    .minus(drivetrain.getPose().getRotation())
                    .getDegrees())) {
          closestRotation =
              Math.abs(
                  getFacePose(ReefFaces.kl)
                      .getRotation()
                      .minus(drivetrain.getPose().getRotation())
                      .getDegrees());
          newFace = ReefFaces.kl;
        }
        break;
      case DISTANCE:
        double closestDistance =
            getFacePose(ReefFaces.ab)
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation());
        if (closestDistance
            > getFacePose(ReefFaces.cd)
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation())) {
          closestDistance =
              getFacePose(ReefFaces.cd)
                  .getTranslation()
                  .getDistance(drivetrain.getPose().getTranslation());
          newFace = ReefFaces.cd;
        }
        if (closestDistance
            > getFacePose(ReefFaces.ef)
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation())) {
          closestDistance =
              getFacePose(ReefFaces.ef)
                  .getTranslation()
                  .getDistance(drivetrain.getPose().getTranslation());
          newFace = ReefFaces.ef;
        }
        if (closestDistance
            > getFacePose(ReefFaces.gh)
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation())) {
          closestDistance =
              getFacePose(ReefFaces.gh)
                  .getTranslation()
                  .getDistance(drivetrain.getPose().getTranslation());
          newFace = ReefFaces.gh;
        }
        if (closestDistance
            > getFacePose(ReefFaces.ij)
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation())) {
          closestDistance =
              getFacePose(ReefFaces.ij)
                  .getTranslation()
                  .getDistance(drivetrain.getPose().getTranslation());
          newFace = ReefFaces.ij;
        }
        if (closestDistance
            > getFacePose(ReefFaces.kl)
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation())) {
          closestDistance =
              getFacePose(ReefFaces.kl)
                  .getTranslation()
                  .getDistance(drivetrain.getPose().getTranslation());
          newFace = ReefFaces.kl;
        }
        break;
    }
    this.targetFace = newFace;
  }

  public void setTargetSide(ReefSide side) {
    this.targetSide = side;
  }

  public boolean isTargetSideLeft() {
    if (DriverStation.isAutonomous() || targetingMethod == TargetingMethod.NO_AUTO_TARGET) {
      return targetSide.isLeft();
    } else {
      return (targetFace.inverted ? !targetSide.isLeft() : targetSide.isLeft());
    }
  }

  public void setTargetSideIfAble(ReefSide side) {
    if (targetingMethod != TargetingMethod.NO_AUTO_TARGET) {
      this.targetSide = side;
    }
  }

  public void setTargets(Reef reef, ReefHeight height) {
    this.targetFace =
        switch (reef) {
          case A, B -> ReefFaces.ab;
          case C, D -> ReefFaces.cd;
          case E, F -> ReefFaces.ef;
          case G, H -> ReefFaces.gh;
          case I, J -> ReefFaces.ij;
          case K, L -> ReefFaces.kl;
        };
    this.targetSide =
        switch (reef) {
          case A, C, E, G, I, K -> ReefSide.left;
          case B, D, F, H, J, L -> ReefSide.right;
        };
    this.targetLevel =
        switch (height) {
          case L2 -> ReefLevel.L2;
          case L3 -> ReefLevel.L3;
          case L4 -> ReefLevel.L4;
        };
  }

  public void setTargets(Source source, SourceDistance distance) {
    this.autoSourceIsLeft = source == Source.LEFT;
    this.targetSourceSide =
        switch (distance) {
          case FAR -> TargetSourceSide.FAR;
          case MID -> TargetSourceSide.MIDDLE;
          case CLOSE -> TargetSourceSide.CLOSE;
        };
  }

  public enum GamePiecePositions {
    NONE(),
    CORAL_IN_FUNNEL(),
    CORAL_IN_MANIPULATOR(),
    CORAL_SECURED_IN_MANIPULATOR(),
    ALGAE_IN_CLAW()
  }

  public enum ReefLevel {
    L1(),
    L2(),
    L3(),
    L4()
  }

  public enum TargetType {
    CORAL(),
    ALGAE()
  }

  public enum AlignTarget {
    SOURCE(),
    REEF(),
    AP()
  }

  private int getTagForFace() {
    return switch (targetFace) {
      case ab -> isRedAlliance ? 7 : 18;
      case cd -> isRedAlliance ? 8 : 17;
      case ef -> isRedAlliance ? 9 : 22;
      case gh -> isRedAlliance ? 10 : 21;
      case ij -> isRedAlliance ? 11 : 20;
      case kl -> isRedAlliance ? 6 : 19;
    };
  }

  private int getTagForFace(ReefFaces face) {
    return switch (face) {
      case ab -> isRedAlliance ? 7 : 18;
      case cd -> isRedAlliance ? 8 : 17;
      case ef -> isRedAlliance ? 9 : 22;
      case gh -> isRedAlliance ? 10 : 21;
      case ij -> isRedAlliance ? 11 : 20;
      case kl -> isRedAlliance ? 6 : 19;
    };
  }

  public void toggleTargetType() {
    this.targetingType = (targetingType == TargetType.ALGAE ? TargetType.CORAL : TargetType.ALGAE);
  }

  public void setTargetSourceSide(TargetSourceSide side) {
    this.targetSourceSide = side;
  }

  public void setTargetLevel(ReefLevel level) {
    this.targetLevel = level;
  }

  public enum TargetSourceSide {
    FAR(),
    MIDDLE(),
    CLOSE()
  }

  public void setTarget(ReefFaces face, ReefSide side) {
    this.targetFace = face;
    this.targetSide = side;
  }

  public enum TargetingMethod {
    ROTATION(),
    DISTANCE(),
    NO_AUTO_TARGET()
  }

  public enum ReefFaces {
    ab(),
    cd(),
    ef(true),
    gh(true),
    ij(true),
    kl();

    public final boolean inverted;

    ReefFaces() {
      this.inverted = false;
    }

    ReefFaces(boolean inverted) {
      this.inverted = inverted;
    }

    public boolean isHighAlgae() {
      return this == ab || this == ef || this == ij;
    }
  }

  public enum ReefSide {
    left(),
    right();

    public boolean isLeft() {
      return this == left;
    }
  }

  @AutoLogOutput(key = "Superstructure/WantedState")
  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public enum WantedState {
    ZERO,
    STOPPED,
    DEFAULT_STATE,
    AUTO_DRIVE_TO_CORAL_STATION,
    INTAKE_CORAL_FROM_STATION,
    AUTO_DRIVE_TO_REEF,
    SCORE_AUTO,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    MANUAL_L4,
    MANUAL_L3,
    MANUAL_L2,
    MANUAL_L1,
    INTAKE,
    INTAKE_ALGAE_FROM_REEF,
    INTAKE_ALGAE_FROM_GROUND,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
    SCORE_ALGAE,
    PRE_CLIMB,
    CLIMB,
    CLIMB_MANUAL
  }

  public enum CurrentState {
    ZERO,
    STOPPED,
    NO_PIECE_TELEOP,
    HOLDING_CORAL_TELEOP,
    NO_PIECE_AUTO,
    HOLDING_CORAL_AUTO,
    HOLDING_ALGAE,
    AUTO_DRIVE_TO_CORAL_STATION,
    INTAKE_CORAL_FROM_STATION,
    INTAKE_CORAL_FROM_STATION_AUTO,
    AUTO_DRIVE_TO_REEF,
    SCORE_TELEOP_L2,
    SCORE_TELEOP_L3,
    SCORE_TELEOP_L4,
    SCORE_AUTO_L2,
    SCORE_AUTO_L3,
    SCORE_AUTO_L4,
    MANUAL_L4,
    MANUAL_L3,
    MANUAL_L2,
    MANUAL_L1,
    INTAKE_ALGAE_FROM_REEF,
    INTAKE_ALGAE_FROM_GROUND,
    MOVE_ALGAE_TO_NET_POSITION,
    SCORE_ALGAE_IN_NET,
    MOVE_ALGAE_TO_PROCESSOR_POSITION,
    SCORE_ALGAE_IN_PROCESSOR,
    PRE_CLIMB,
    CLIMB,
    CLIMB_MANUAL
  }

  public void stopGamePieceScore() {
    switch (currentState) {
      case SCORE_ALGAE_IN_NET, SCORE_ALGAE_IN_PROCESSOR:
        setWantedState(WantedState.SCORE_ALGAE);
        break;
      default:
        scoreCoralFlag = false;
        manualScoreCoralBeingFlagged = false;
        break;
    }
  }

  public void decideGamePieceScore() {
    switch (currentState) {
      case MOVE_ALGAE_TO_NET_POSITION:
        setWantedState(WantedState.SCORE_ALGAE_IN_NET);
        break;
      case MOVE_ALGAE_TO_PROCESSOR_POSITION:
        setWantedState(WantedState.SCORE_ALGAE_IN_PROCESSOR);
        break;
      default:
        scoreCoralFlag = true;
        manualScoreCoralBeingFlagged = true;
        break;
    }
  }

  public boolean onOtherHalfOfField() {
    return isRedAlliance
        ? (drivetrain.getPose().getMeasureX().in(Meters)
            <= FieldConstants.fieldLength.in(Meters) / 2)
        : (drivetrain.getPose().getMeasureX().in(Meters)
            >= FieldConstants.fieldLength.in(Meters) / 2);
  }

  public boolean onLeftHalfOfField() {
    return isRedAlliance
        ? (drivetrain.getPose().getMeasureY().in(Meters)
            <= (FieldConstants.fieldWidth.in(Meters) / 2) + 0.5)
        : (drivetrain.getPose().getMeasureY().in(Meters)
            >= (FieldConstants.fieldWidth.in(Meters) / 2) - 0.5);
  }

  public CurrentState decideStateForAlgae() {
    return onOtherHalfOfField()
        ? CurrentState.MOVE_ALGAE_TO_PROCESSOR_POSITION
        : onLeftHalfOfField()
            ? CurrentState.MOVE_ALGAE_TO_NET_POSITION
            : CurrentState.MOVE_ALGAE_TO_PROCESSOR_POSITION;
  }

  public Command setWantedStateCommand(WantedState state) {
    return Commands.runOnce(() -> setWantedState(state));
  }

  /**
   * @param hasCoral WantedState when this button is pressed and the manipulator detects coral
   * @param hasAlgae WantedState when this button is pressed and the claw has algae
   * @param noPiece WantedState when this button is pressed and there are no game pieces detected
   */
  public Command configureButtonBinding(
      WantedState hasCoral, WantedState hasAlgae, WantedState noPiece) {
    return Commands.either(
        setWantedStateCommand(hasCoral),
        Commands.either(
            setWantedStateCommand(hasAlgae), setWantedStateCommand(noPiece), claw::hasAlgae),
        manipulator::detectsCoral);
  }
}
