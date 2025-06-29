// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutomationLevel;
import frc.robot.Constants.ReefSelectionMethod;
import frc.robot.Constants.ScoringSide;
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
import frc.robot.utils.TunableController;
import frc.robot.utils.TargetingComputer.Targets;

public class Superstructure extends SubsystemBase {
    private final Drive drivetrain;
    private final Claw claw;
    private final Climber climber;
    private final Elevator elevator;
    private final Funnel funnel;
    private final LEDSubsystem ledSubsystem;
    private final Manipulator manipulator;
    private final Vision vision;

    private final TunableController driver;
    private final TunableController mech;

    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;
    private CurrentState previousState;

    private AutomationLevel automationLevel = AutomationLevel.AUTO_RELEASE;
    private ReefSelectionMethod reefSelectionMethod = ReefSelectionMethod.POSE;

    private final AutomationLevelChooser operatorDashboard;

    private boolean bump = false;

    private boolean isRedAlliance = false;

    private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.times(0.025))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.025)) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private PIDController rotation = new PIDController(0, 0, 0);
    private PIDController translationx = new PIDController(0, 0, 0);
    private PIDController translationy = new PIDController(0, 0, 0);

    public Superstructure(Drive drivetrain,
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
        this.operatorDashboard = new AutomationLevelChooser();
    }

    public void setAlliance(boolean redAlliance) {
        isRedAlliance = redAlliance;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Superstructure/ReefSelectionMethod", reefSelectionMethod);

        automationLevel = operatorDashboard.getAutomationLevel();

        Logger.recordOutput("Superstructure/WantedState", wantedState);
        Logger.recordOutput("Superstructure/currentState", currentState);
        Logger.recordOutput("Superstructure/PreviousState", previousState);

        currentState = handStateTransitions();
        applyStates();
    }

    private CurrentState handStateTransitions() {
        previousState = currentState;
        switch (wantedState) {
            case ZERO:
                currentState = CurrentState.ZERO;
                break;
            case INTAKE_CORAL_FROM_STATION:
                currentState = CurrentState.INTAKE_CORAL_FROM_STATION;
                break;
            case DEFAULT_STATE:
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
            case SCORE_L1_MANUAL_ALIGN:
                currentState = CurrentState.SCORE_TELEOP_L1_MANUAL_ALIGNMENT;
                break;
            case SCORE_LEFT_L2:
                currentState = DriverStation.isAutonomous()
                        ? CurrentState.SCORE_LEFT_AUTO_L2
                        : CurrentState.SCORE_LEFT_TELEOP_L2;
                break;
            case SCORE_LEFT_L3:
                currentState = DriverStation.isAutonomous()
                        ? CurrentState.SCORE_LEFT_AUTO_L3
                        : CurrentState.SCORE_LEFT_TELEOP_L3;
                break;
            case SCORE_LEFT_L4:
                currentState = DriverStation.isAutonomous()
                        ? CurrentState.SCORE_LEFT_AUTO_L4
                        : CurrentState.SCORE_LEFT_TELEOP_L4;
                break;
            case SCORE_RIGHT_L2:
                currentState = DriverStation.isAutonomous()
                        ? CurrentState.SCORE_RIGHT_AUTO_L2
                        : CurrentState.SCORE_RIGHT_TELEOP_L2;
                break;
            case SCORE_RIGHT_L3:
                currentState = DriverStation.isAutonomous()
                        ? CurrentState.SCORE_RIGHT_AUTO_L3
                        : CurrentState.SCORE_RIGHT_TELEOP_L3;
                break;
            case SCORE_RIGHT_L4:
                currentState = DriverStation.isAutonomous()
                        ? CurrentState.SCORE_RIGHT_AUTO_L4
                        : CurrentState.SCORE_RIGHT_TELEOP_L4;
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
            case CLIMB:
                currentState = CurrentState.CLIMB;
                break;
            default:
                currentState = CurrentState.STOPPED;
                break;
        }
        return currentState;
    }

    private void applyStates() {
        if (previousState == CurrentState.CLIMB && currentState != CurrentState.CLIMB) {
            climberSubsystem.setWantedState(ClimberSubsystem.WantedState.STOWED);
            hasArmReachedIntermediateClimbPosition = false;
        }
        if (previousState != CurrentState.INTAKE_ALGAE_FROM_REEF
                && currentState == CurrentState.INTAKE_ALGAE_FROM_REEF
                && armSubsystem.getCurrentExtensionPositionInMeters() > Units.inchesToMeters(30.0)) {
            doesArmNeedToGoToIntermediatePoseForReefAlgaePickup = true;
            hasArmReachedIntermediatePoseForReefAlgaePickup = false;
            hasDriveReachedIntermediatePoseForReefAlgaePickup = false;
        } else if (currentState == CurrentState.INTAKE_ALGAE_FROM_REEF
                && (previousState == CurrentState.SCORE_LEFT_TELEOP_L2
                        || previousState == CurrentState.SCORE_RIGHT_TELEOP_L2)) {
            doesArmNeedToGoToIntermediatePoseForReefAlgaePickup = true;
            hasArmReachedIntermediatePoseForReefAlgaePickup = false;
            hasDriveReachedIntermediatePoseForReefAlgaePickup = false;
        } else if (previousState == CurrentState.INTAKE_ALGAE_FROM_REEF
                && currentState != CurrentState.INTAKE_ALGAE_FROM_REEF) {
            doesArmNeedToGoToIntermediatePoseForReefAlgaePickup = false;
        }
        switch (currentState) {
            case ZERO:
                zero();
                break;
            case INTAKE_CORAL_FROM_STATION:
                intakeCoralFromStation();
                break;
            case NO_PIECE_TELEOP:
                noPiece();
                break;
            case NO_PIECE_AUTO:
                noPiece();
                break;
            case HOLDING_CORAL_AUTO:
                holdingCoralAuto();
                break;
            case HOLDING_CORAL_TELEOP:
                holdingCoral();
                break;
            case HOLDING_ALGAE:
                holdingAlgae();
                break;
            case SCORE_TELEOP_L1_MANUAL_ALIGNMENT:
                scoreL1Teleop();
                break;
            case SCORE_LEFT_TELEOP_L2:
                scoreL2Teleop(ScoringSide.LEFT);
                break;
            case SCORE_LEFT_TELEOP_L3:
                scoreL3Teleop(ScoringSide.LEFT);
                break;
            case SCORE_LEFT_TELEOP_L4:
                scoreL4Teleop(ScoringSide.LEFT);
                break;
            case SCORE_RIGHT_TELEOP_L2:
                scoreL2Teleop(ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_TELEOP_L3:
                scoreL3Teleop(ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_TELEOP_L4:
                scoreL4Teleop(ScoringSide.RIGHT);
                break;
            case SCORE_LEFT_AUTO_L2:
                scoreL2Auto(ScoringSide.LEFT);
                break;
            case SCORE_LEFT_AUTO_L3:
                scoreL3Auto(ScoringSide.LEFT);
                break;
            case SCORE_LEFT_AUTO_L4:
                scoreL4Auto(ScoringSide.LEFT);
                break;
            case SCORE_RIGHT_AUTO_L2:
                scoreL2Auto(ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_AUTO_L3:
                scoreL3Auto(ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_AUTO_L4:
                scoreL4Auto(ScoringSide.RIGHT);
                break;
            case MANUAL_L4:
                ejectL4();
                break;
            case MANUAL_L3:
                ejectL3();
                break;
            case MANUAL_L2:
                ejectL2();
                break;
            case MANUAL_L1:
                ejectL1();
                break;
            case INTAKE_ALGAE_FROM_REEF:
                intakeAlgaeFromReef();
                break;
            case INTAKE_ALGAE_FROM_GROUND:
                intakeAlgaeFromGround();
                break;
            case SCORE_ALGAE_IN_NET:
                scoreAlgaeNet();
                break;
            case SCORE_ALGAE_IN_PROCESSOR:
                scoreAlgaeProcessor();
                break;
            case MOVE_ALGAE_TO_NET_POSITION:
                moveAlgaeToNetPosition();
                break;
            case MOVE_ALGAE_TO_PROCESSOR_POSITION:
                moveAlgaeToProcessorPosition();
                break;
            case CLIMB:
                automatedCLimb();
                break;
            case STOPPED:
                stopped();
                break;
        }
    }

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
    }

    private void intakeCoralFromStation() {
        manipulator.setState(bump ? ManipulatorStates.BUMP : ManipulatorStates.INTAKE);
        funnel.setState(
                bump ? FunnelState.BUMP : manipulator.detectsCoral() ? FunnelState.INTAKE_SLOW : FunnelState.INTAKE);
        applyDrive(sourceRotation(drivetrain.getPose()), driver.customLeft().getX(), driver.customLeft().getY());
    }

    private void noPiece() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.INTAKE);
        funnel.setState(FunnelState.OFF);
        manipulator.setState(ManipulatorStates.OFF);
        applyDrive(driver);
    }

    private void applyDrive(TunableController controller) {
        drivetrain.applyRequest(
                () -> drive
                        .withVelocityX(
                                MaxSpeed.times(
                                        -controller.customLeft().getY()))
                        .withVelocityY(
                                MaxSpeed.times(
                                        -controller.customLeft().getX()))
                        .withRotationalRate(
                                Constants.MaxAngularRate.times(controller.customRight().getX())))
                .schedule();
    }

    private void applyDrive(Rotation2d rotationSnap, double x, double y) {
        drivetrain.applyRequest(
                () -> drive
                        .withVelocityX(
                                MaxSpeed.times(
                                        -y))
                        .withVelocityY(
                                MaxSpeed.times(
                                        -x))
                        .withRotationalRate(
                                Constants.MaxAngularRate.times(rotation.calculate(
                                        drivetrain.getPose().getRotation().minus(rotationSnap).getDegrees(), 0))))
                .schedule();
    }

    public Rotation2d sourceRotation(Pose2d pose) {
        return Rotation2d.fromDegrees(pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2
                ? isRedAlliance ? 234 : 54
                : isRedAlliance ? 126 : 306);
    }

    public enum WantedState {
        ZERO, STOPPED, DEFAULT_STATE, INTAKE_CORAL_FROM_STATION, SCORE_L1_MANUAL_ALIGN, SCORE_LEFT_L2, SCORE_LEFT_L3, SCORE_LEFT_L4, SCORE_RIGHT_L2, SCORE_RIGHT_L3, SCORE_RIGHT_L4, MANUAL_L4, MANUAL_L3, MANUAL_L2, MANUAL_L1, INTAKE_ALGAE_FROM_REEF, INTAKE_ALGAE_FROM_GROUND, MOVE_ALGAE_TO_NET_POSITION, SCORE_ALGAE_IN_NET, MOVE_ALGAE_TO_PROCESSOR_POSITION, SCORE_ALGAE_IN_PROCESSOR, CLIMB
    }

    public enum CurrentState {
        ZERO, STOPPED, NO_PIECE_TELEOP, HOLDING_CORAL_TELEOP, NO_PIECE_AUTO, HOLDING_CORAL_AUTO, HOLDING_ALGAE, INTAKE_CORAL_FROM_STATION, SCORE_TELEOP_L1_MANUAL_ALIGNMENT, SCORE_LEFT_TELEOP_L2, SCORE_LEFT_TELEOP_L3, SCORE_LEFT_TELEOP_L4, SCORE_RIGHT_TELEOP_L2, SCORE_RIGHT_TELEOP_L3, SCORE_RIGHT_TELEOP_L4, SCORE_LEFT_AUTO_L2, SCORE_LEFT_AUTO_L3, SCORE_LEFT_AUTO_L4, SCORE_RIGHT_AUTO_L2, SCORE_RIGHT_AUTO_L3, SCORE_RIGHT_AUTO_L4, MANUAL_L4, MANUAL_L3, MANUAL_L2, MANUAL_L1, INTAKE_ALGAE_FROM_REEF, INTAKE_ALGAE_FROM_GROUND, MOVE_ALGAE_TO_NET_POSITION, SCORE_ALGAE_IN_NET, MOVE_ALGAE_TO_PROCESSOR_POSITION, SCORE_ALGAE_IN_PROCESSOR, CLIMB
    }
}
