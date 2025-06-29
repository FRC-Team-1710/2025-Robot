// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.fasterxml.jackson.annotation.JsonInclude.Include;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutomationLevel;
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
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TunableController;
import frc.robot.utils.TargetingComputer.Levels;
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

    private ReefFaces targetFace = ReefFaces.ab;
    private ReefSide targetSide = ReefSide.left;

    private AutomationLevel automationLevel = AutomationLevel.AUTO_RELEASE;

    private final AutomationLevelChooser operatorDashboard;

    private boolean bump = false;

    private boolean scoreCoralFlag = false;
    private boolean scoreAlgaeFlag = false;

    private boolean isRedAlliance = false;

    private final double offsetX = 17.5;
    private final double offsetY = 7;

    private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.times(0.025))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.025)) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
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
                holdingCoral();
                break;
            case HOLDING_CORAL_TELEOP:
                holdingCoral();
                break;
            case HOLDING_ALGAE:
                holdingAlgae();
                break;
            case SCORE_LEFT_TELEOP_L2:
                scoreL2Teleop();
                break;
            case SCORE_LEFT_TELEOP_L3:
                scoreL3Teleop();
                break;
            case SCORE_LEFT_TELEOP_L4:
                scoreL4Teleop();
                break;
            case SCORE_RIGHT_TELEOP_L2:
                scoreL2Teleop();
                break;
            case SCORE_RIGHT_TELEOP_L3:
                scoreL3Teleop();
                break;
            case SCORE_RIGHT_TELEOP_L4:
                scoreL4Teleop();
                break;
            case SCORE_LEFT_AUTO_L2:
                scoreL2Auto();
                break;
            case SCORE_LEFT_AUTO_L3:
                scoreL3Auto();
                break;
            case SCORE_LEFT_AUTO_L4:
                scoreL4Auto();
                break;
            case SCORE_RIGHT_AUTO_L2:
                scoreL2Auto();
                break;
            case SCORE_RIGHT_AUTO_L3:
                scoreL3Auto();
                break;
            case SCORE_RIGHT_AUTO_L4:
                scoreL4Auto();
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
                climb();
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

        if (claw.getState() == ClawStates.IDLE && elevator.getState() == ElevatorStates.INTAKE) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }

    private void intakeCoralFromStation() {
        scoreCoralFlag = false;
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.INTAKE);
        manipulator.setState(bump ? ManipulatorStates.BUMP : ManipulatorStates.INTAKE);
        funnel.setState(
                bump ? FunnelState.BUMP : manipulator.detectsCoral() ? FunnelState.INTAKE_SLOW : FunnelState.INTAKE);
        applyDrive(sourceRotation(drivetrain.getPose()), driver.customLeft().getX(), driver.customLeft().getY(),
                driver.customRight().getX());
    }

    private void noPiece() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.INTAKE);
        funnel.setState(FunnelState.OFF);
        manipulator.setState(ManipulatorStates.OFF);
        applyDrive(driver);
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
        applyDrive(driver);
    }

    private void holdingAlgae() {
        claw.setState(ClawStates.HOLD);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.INTAKE);
        funnel.setState(FunnelState.OFF);
        manipulator.setState(ManipulatorStates.OFF);
        applyDrive(driver);
    }

    private void scoreL2Teleop() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L2);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = ((isInRange() && automationLevel == AutomationLevel.AUTO_RELEASE && elevator.isAtTarget())
                || scoreCoralFlag);
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(getTargetPose(), driver.customLeft().getX(), driver.customLeft().getY(),
                driver.customRight().getX());
        if (!manipulator.detectsCoral()) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }

    private void scoreL3Teleop() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L3);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = ((isInRange() && automationLevel == AutomationLevel.AUTO_RELEASE && elevator.isAtTarget())
                || scoreCoralFlag);
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(getTargetPose(), driver.customLeft().getX(), driver.customLeft().getY(),
                driver.customRight().getX());
        if (!manipulator.detectsCoral()) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }

    private void scoreL4Teleop() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L4);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = ((isInRange() && automationLevel == AutomationLevel.AUTO_RELEASE && elevator.isAtTarget())
                || scoreCoralFlag);
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(getTargetPose(), driver.customLeft().getX(), driver.customLeft().getY(),
                driver.customRight().getX());
        if (!manipulator.detectsCoral()) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }

    private void scoreL2Auto() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L2);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = ((isInRange() && elevator.isAtTarget()) || scoreCoralFlag);
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(getTargetPose(), driver.customLeft().getX(), driver.customLeft().getY(),
                driver.customRight().getX());
        if (!manipulator.detectsCoral()) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }

    private void scoreL3Auto() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L3);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = ((isInRange() && elevator.isAtTarget()) || scoreCoralFlag);
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(getTargetPose(), driver.customLeft().getX(), driver.customLeft().getY(),
                driver.customRight().getX());
        if (!manipulator.detectsCoral()) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }

    private void scoreL4Auto() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L4);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = ((isInRange() && elevator.isAtTarget()) || scoreCoralFlag);
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(getTargetPose(), driver.customLeft().getX(), driver.customLeft().getY(),
                driver.customRight().getX());
        if (!manipulator.detectsCoral()) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }

    private void ejectL1() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L1);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = true;
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(driver);
    }

    private void ejectL2() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L2);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = true;
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(driver);
    }

    private void ejectL3() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L3);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = true;
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(driver);
    }

    private void ejectL4() {
        claw.setState(ClawStates.IDLE);
        climber.setState(ClimberStates.STOWED);
        elevator.setState(ElevatorStates.L4);
        funnel.setState(FunnelState.OFF);
        scoreCoralFlag = true;
        manipulator.setState(scoreCoralFlag ? ManipulatorStates.OUTTAKE : ManipulatorStates.OFF);
        applyDrive(driver);
    }

    private void intakeAlgaeFromReef() {
        climber.setState(ClimberStates.STOWED);
        funnel.setState(FunnelState.OFF);
        manipulator.setState(ManipulatorStates.OFF);
        if (claw.getState() != ClawStates.GRAB || !claw.isAtTarget() || elevator.getState() != (targetFace.isHighAlgae() ? ElevatorStates.ALGAE_HIGH : ElevatorStates.ALGAE_LOW) || !elevator.isAtTarget()) {
            applyDrive(getBeforeReadyToGrabAlgaePose());
            if (drivetrain.getPose().getTranslation().getDistance(getBeforeReadyToGrabAlgaePose().getTranslation()) < Units.inchesToMeters(2.5)) {
                elevator.setState(targetFace.isHighAlgae() ? ElevatorStates.ALGAE_HIGH : ElevatorStates.ALGAE_LOW);
                claw.setState(ClawStates.GRAB);
            }
        } else if (!claw.hasAlgae()) {
            applyDrive(getReadyToGrabAlgaePose());
        } else if (claw.hasAlgae()) {
            applyDrive(getBeforeReadyToGrabAlgaePose());
            if (drivetrain.getPose().getTranslation().getDistance(getBeforeReadyToGrabAlgaePose().getTranslation()) < Units.inchesToMeters(2.5)) {
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
            applyDrive(Units.degreesToRadians(vision.getAlgaeYaw()) * 0.4, driver);
        } else {
            applyDrive(driver);
        }
        if (claw.hasAlgae()) {
            setWantedState(WantedState.DEFAULT_STATE);
        }
    }













    
    
        private void stopped() {
            claw.setState(ClawStates.STOP);
            climber.setState(0);
            elevator.setState(ElevatorStates.STOP);
            funnel.setState(FunnelState.STOP);
            manipulator.setState(ManipulatorStates.OFF);
        }

    private void applyDrive(double rotation, TunableController controller) {
        drivetrain.applyRequest(
                () -> fieldCentric
                        .withVelocityX(
                                MaxSpeed.times(
                                        -controller.customLeft().getY()))
                        .withVelocityY(
                                MaxSpeed.times(
                                        -controller.customLeft().getX()))
                        .withRotationalRate(
                                Constants.MaxAngularRate.times(rotation + (controller.customRight().getX()*0.25))))
                .schedule();
    }

    private void applyDrive(TunableController controller) {
        drivetrain.applyRequest(
                () -> fieldCentric
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

    private void applyDrive(Rotation2d rotationSnap, double x, double y, double rot) {
        drivetrain.applyRequest(
                () -> fieldCentric
                        .withVelocityX(
                                MaxSpeed.times(
                                        -y))
                        .withVelocityY(
                                MaxSpeed.times(
                                        -x))
                        .withRotationalRate(
                                Constants.MaxAngularRate.times(rotation.calculate(
                                        drivetrain.getPose().getRotation().minus(rotationSnap).getDegrees(), 0)
                                        + (rot * 0.25))))
                .schedule();
    }

    private void applyDrive(Pose2d pose, double x, double y, double rot) {
        drivetrain.applyRequest(
                () -> fieldCentric
                        .withVelocityX(
                                MaxSpeed.times(
                                        -(translationx.calculate(drivetrain.getPose().getX() - pose.getX(), 0)
                                                + (y * 0.25))))
                        .withVelocityY(
                                MaxSpeed.times(
                                        -(translationy.calculate(drivetrain.getPose().getY() - pose.getY(), 0)
                                                + (x * 0.25))))
                        .withRotationalRate(
                                Constants.MaxAngularRate.times(rotation.calculate(
                                        drivetrain.getPose().getRotation().minus(pose.getRotation()).getDegrees(), 0)
                                        + (rot * 0.25))))
                .schedule();
    }

    private void applyDrive(Pose2d pose) {
        drivetrain.applyRequest(
                () -> fieldCentric
                        .withVelocityX(
                                MaxSpeed.times(
                                        -(translationx.calculate(drivetrain.getPose().getX() - pose.getX(), 0)
                                                + (driver.customLeft().getY() * 0.25))))
                        .withVelocityY(
                                MaxSpeed.times(
                                        -(translationy.calculate(drivetrain.getPose().getY() - pose.getY(), 0)
                                                + (driver.customLeft().getX() * 0.25))))
                        .withRotationalRate(
                                Constants.MaxAngularRate.times(rotation.calculate(
                                        drivetrain.getPose().getRotation().minus(pose.getRotation()).getDegrees(), 0)
                                        + (driver.customRight().getX() * 0.25))))
                .schedule();
    }

    public boolean isInRange() {
        return (Math.abs(drivetrain.getPose().getTranslation().getDistance(getTargetPose().getTranslation())) < Units
                .inchesToMeters(1.5)
                && Math.abs(drivetrain.getRotation().minus(getTargetPose().getRotation()).getDegrees()) < 1.5);
    }

    public Rotation2d sourceRotation(Pose2d pose) {
        return Rotation2d.fromDegrees(pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2
                ? isRedAlliance ? 234 : 54
                : isRedAlliance ? 126 : 306);
    }

    public Pose2d getTargetPose() {
        return new Pose2d(
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
                                                offsetX,
                                                targetSide.isLeft() ? -offsetY : offsetY,
                                                new Rotation2d(Math.PI)));
    }

    public Pose2d getBeforeReadyToGrabAlgaePose() {
        return new Pose2d(
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
                                                Units.inchesToMeters(32),
                                                0,
                                                new Rotation2d(Math.PI)));
    }

    public Pose2d getReadyToGrabAlgaePose() {
        return new Pose2d(
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
                                                Units.inchesToMeters(23),
                                                0,
                                                new Rotation2d(Math.PI)));
    }

    public int getTagForFace() {
        return switch (targetFace) {
            case ab -> isRedAlliance ? 7 : 18;
            case cd -> isRedAlliance ? 8 : 17;
            case ef -> isRedAlliance ? 9 : 22;
            case gh -> isRedAlliance ? 10 : 21;
            case ij -> isRedAlliance ? 11 : 20;
            case kl -> isRedAlliance ? 6 : 19;
        };
    }

    public void setTarget(ReefFaces face, ReefSide side) {
        this.targetFace = face;
        this.targetSide = side;
    }

    public enum ReefFaces {
        ab(), cd(), ef(), gh(), ij(), kl();

        ReefFaces() {
        }

        public boolean isHighAlgae() {
            return this == ab || this == ef || this == ij;
        }
    }

    public enum ReefSide {
        left(), right();

        public boolean isLeft() {
            return this == left;
        }
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public enum WantedState {
        ZERO, STOPPED, DEFAULT_STATE, INTAKE_CORAL_FROM_STATION, SCORE_LEFT_L2, SCORE_LEFT_L3, SCORE_LEFT_L4, SCORE_RIGHT_L2, SCORE_RIGHT_L3, SCORE_RIGHT_L4, MANUAL_L4, MANUAL_L3, MANUAL_L2, MANUAL_L1, INTAKE_ALGAE_FROM_REEF, INTAKE_ALGAE_FROM_GROUND, MOVE_ALGAE_TO_NET_POSITION, SCORE_ALGAE_IN_NET, MOVE_ALGAE_TO_PROCESSOR_POSITION, SCORE_ALGAE_IN_PROCESSOR, CLIMB
    }

    public enum CurrentState {
        ZERO, STOPPED, NO_PIECE_TELEOP, HOLDING_CORAL_TELEOP, NO_PIECE_AUTO, HOLDING_CORAL_AUTO, HOLDING_ALGAE, INTAKE_CORAL_FROM_STATION, SCORE_LEFT_TELEOP_L2, SCORE_LEFT_TELEOP_L3, SCORE_LEFT_TELEOP_L4, SCORE_RIGHT_TELEOP_L2, SCORE_RIGHT_TELEOP_L3, SCORE_RIGHT_TELEOP_L4, SCORE_LEFT_AUTO_L2, SCORE_LEFT_AUTO_L3, SCORE_LEFT_AUTO_L4, SCORE_RIGHT_AUTO_L2, SCORE_RIGHT_AUTO_L3, SCORE_RIGHT_AUTO_L4, MANUAL_L4, MANUAL_L3, MANUAL_L2, MANUAL_L1, INTAKE_ALGAE_FROM_REEF, INTAKE_ALGAE_FROM_GROUND, MOVE_ALGAE_TO_NET_POSITION, SCORE_ALGAE_IN_NET, MOVE_ALGAE_TO_PROCESSOR_POSITION, SCORE_ALGAE_IN_PROCESSOR, CLIMB
    }
}
