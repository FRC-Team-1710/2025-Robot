// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorToTargetLevel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.TargetingComputer;
import frc.robot.utils.TargetingComputer.Levels;
import frc.robot.utils.TunableController;
import org.littletonrobotics.junction.Logger;

public class StateHandler extends SubsystemBase {
  private States state = States.Idle;

  private States schedueledState = null;

  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;

  private final SwerveRequest.FieldCentric driveTrain =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.025))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.025)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentric =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed.times(0.025))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.025))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private Translation2d driverLeft;
  private Translation2d driverRight;

  double alignP = 1;
  double rotP = .75;

  private Elevator elevator;
  private Claw claw;
  private Climber climber;
  private Drive drive;
  private Funnel funnel;
  private Manipulator manipulator;
  private Vision vision;
  private TunableController driver;
  private TunableController mech;
  private Command elevatorToTargetLevel;

  /** Creates a new StateHandler. */
  public StateHandler(
      Elevator elevator,
      Claw claw,
      Climber climber,
      Drive drive,
      Funnel funnel,
      Manipulator manipulator,
      Vision vision,
      TunableController driver,
      TunableController mech) {
    this.elevator = elevator;
    this.claw = claw;
    this.climber = climber;
    this.drive = drive;
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.vision = vision;
    this.driver = driver;
    this.mech = mech;
    elevatorToTargetLevel = new ElevatorToTargetLevel(elevator);
  }

  public void resetToIdle() {
    state = States.Idle;
  }

  public enum States {
    Idle,
    AlgaeIdle,
    IntakeFromSource,
    PrepCoralPlace,
    PlaceCoral,
    PrepAlgaeGrab,
    GrabAlgaeOffReef,
    EjectAlgae,
    Net,
    Processer,
    PrepClimb,
    Climb,
    UnClimb,
    PlaceAlgae;
  }

  public void schedualState(States to) {
    schedueledState = to;
  }

  public boolean gettingAlgae() {
    if (state == States.AlgaeIdle
        || state == States.GrabAlgaeOffReef
        || state == States.PrepAlgaeGrab) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() { // VERY GOOD LOGIC, TRUST
    Logger.recordOutput("Current State", state);
    Logger.recordOutput("Next State", schedueledState);
    driverLeft = driver.customLeft();
    driverRight = driver.customRight();
    if (state != schedueledState) { // Checks if current state is not the schedueled state
      if (state == States.Idle
          && (schedueledState == States.IntakeFromSource
              || schedueledState == States.PrepClimb
              || schedueledState
                  == States
                      .PrepCoralPlace)) { // Checks if the state can be swiched to the schedueled
        // state
        state = schedueledState;
      } else if (state == States.AlgaeIdle
          && (schedueledState == States.EjectAlgae
              || schedueledState == States.Net
              || schedueledState == States.Processer)) {
        state = schedueledState;
      } else if (state == States.IntakeFromSource && (schedueledState == States.Idle || schedueledState == States.PrepCoralPlace)) {
        state = schedueledState;
      } else if (state == States.PrepCoralPlace
          && (schedueledState == States.PlaceCoral
              || schedueledState == States.Idle
              || schedueledState == States.PrepAlgaeGrab || schedueledState == States.IntakeFromSource)) {
        state = schedueledState;
      } else if (state == States.PlaceCoral
          && (schedueledState == States.Idle
              || schedueledState == States.PrepCoralPlace
              || schedueledState == States.PrepAlgaeGrab)) {
        state = schedueledState;
      } else if (state == States.PrepAlgaeGrab && (schedueledState == States.GrabAlgaeOffReef)) {
        state = schedueledState;
      } else if (state == States.GrabAlgaeOffReef && (schedueledState == States.AlgaeIdle)) {
        state = schedueledState;
      } else if (state == States.EjectAlgae && (schedueledState == States.Idle)) {
        state = schedueledState;
      } else if (state == States.Net && (schedueledState == States.PlaceAlgae || schedueledState == States.AlgaeIdle)) {
        state = schedueledState;
      } else if (state == States.Processer && (schedueledState == States.PlaceAlgae || schedueledState == States.AlgaeIdle)) {
        state = schedueledState;
      } else if (state == States.PlaceAlgae && (schedueledState == States.Idle || schedueledState == States.AlgaeIdle)) {
        state = schedueledState;
      } else if (state == States.PrepClimb
          && (schedueledState == States.Climb
              || schedueledState == States.UnClimb
              || schedueledState == States.Idle)) {
        state = schedueledState;
      } else if (state == States.Climb
          && (schedueledState == States.PrepClimb || schedueledState == States.UnClimb)) {
        state = schedueledState;
      } else if (state == States.UnClimb
          && (schedueledState == States.PrepClimb || schedueledState == States.Climb)) {
        state = schedueledState;
      }
    }
    runSubsystems(); // Run functions of that state for subsystems
  }

  private void runSubsystems() {
    drivetrainRequestHandler(); // Targets reef/source if state requires it
    if (state == States.Idle) {
      moveToIdle();
    } else if (state == States.AlgaeIdle) {
      moveToAlgaeIdle();
    } else if (state == States.IntakeFromSource) {
      moveToIntakeFromSource();
    } else if (state == States.PrepCoralPlace) {
      moveToPrepCoralPlace();
    } else if (state == States.PlaceCoral) {
      moveToPlaceCoral();
    } else if (state == States.PrepAlgaeGrab) {
      moveToPrepAlgaeGrab();
    } else if (state == States.GrabAlgaeOffReef) {
      moveToGrabAlgaeOffReef();
    } else if (state == States.EjectAlgae) {
      moveToEjectAlgae();
    } else if (state == States.Net) {
      moveToNet();
    } else if (state == States.Processer) {
      moveToProcesser();
    } else if (state == States.PrepClimb) {
      moveToPrepClimb();
    } else if (state == States.Climb) {
      moveToClimb();
    } else if (state == States.UnClimb) {
      moveToUnClimb();
    } else if (state == States.PlaceAlgae) {
      moveToPlaceAlgae();
    }
  }

  private void moveToIdle() {
    elevator.Intake().schedule();
    claw.setEject(0);
    claw.Idle().schedule();
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToClimb() {
    elevator.Intake().schedule();
    claw.Idle().schedule();
    manipulator.Off();
    climber.Down();
    // funnel.Up();
    funnel.Off();
  }

  private void moveToUnClimb() {
    elevator.Intake().schedule();
    claw.Idle().schedule();
    manipulator.Off();
    climber.Up();
    // funnel.Up();
    funnel.Off();
  }

  private void moveToPrepClimb() {
    elevator.Intake().schedule();
    claw.Idle().schedule();
    manipulator.Off();
    climber.Off();
    // funnel.Up();
    funnel.Off();
  }

  private void moveToNet() {
    elevator.L4().schedule();
    claw.NET().schedule();
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToProcesser() {
    elevator.Intake().schedule();
    claw.PROCESSER().schedule();
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToPlaceAlgae() {
    if (claw.isAtTarget() && elevator.isAtTarget()) {
      claw.setEject(0.25);
    }
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToEjectAlgae() {
    claw.setEject(0.25);
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToAlgaeIdle() {
    TargetingComputer.setTargetingAlgae(false);
    TargetingComputer.setReadyToGrabAlgae(false);
    elevator.Intake().schedule();
    claw.AlgaeIdle().schedule();
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToPlaceCoral() {
    elevatorToTargetLevel.schedule();
    if (elevator.isAtTarget()) {
      manipulator.Place();
    }
    claw.Idle().schedule();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToIntakeFromSource() {
    elevator.Intake().schedule();
    claw.Idle().schedule();
    manipulator.Intake();
    climber.Off();
    // funnel.Down();
    if (!manipulator.isCoralIn()) {
      funnel.Intake();
    } else if (manipulator.isCoralIn() && !manipulator.isCoralSecure()) {
      funnel.Inside();
    } else {
      funnel.Off();
    }
  }

  private void moveToPrepCoralPlace() {
    if (vision.getDistanceToTag(TargetingComputer.getCurrentTargetBranch().getApriltag()) < 2
        && (vision.containsRequestedTarget(TargetingComputer.getCurrentTargetBranch().getApriltag())
            || Math.abs(
                    new Rotation2d(
                            Units.degreesToRadians(
                                TargetingComputer.getCurrentTargetBranch().getTargetingAngle()))
                        .minus(drive.getPose().getRotation())
                        .getDegrees())
                > 5)) {
      elevatorToTargetLevel.schedule();
    } else {
      elevator.Intake().schedule();
    }
    claw.Idle().schedule();
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToGrabAlgaeOffReef() {
    if (!claw.isAlgaeIn()) {
      TargetingComputer.setReadyToGrabAlgae(true);
      elevatorToTargetLevel.schedule();
      claw.INTAKE().schedule();
    } else if (claw.isAlgaeIn()) {
      TargetingComputer.setReadyToGrabAlgae(false);
      new ElevatorToTargetLevel(elevator).schedule();
      claw.INTAKE().schedule();
    }
    if (vision.getDistanceToTag(TargetingComputer.getCurrentTargetBranch().getApriltag()) > 0.75
        && claw.isAlgaeIn()) {
      TargetingComputer.setTargetingAlgae(false);
      claw.AlgaeIdle().schedule();
      elevator.Intake().schedule();
      state = States.AlgaeIdle;
    }
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void moveToPrepAlgaeGrab() {
    if (TargetingComputer.getCurrentTargetLevel() == Levels.ALGAE_HIGH
        || TargetingComputer.getCurrentTargetLevel() == Levels.ALGAE_LOW) {
      elevatorToTargetLevel.schedule();
      if (!elevator.isAtTarget() || (!claw.targetingReef() || !claw.isAtTarget())) {
        if (vision.getDistanceToTag(TargetingComputer.getCurrentTargetBranch().getApriltag())
            > 0.75) {
          claw.REEF().schedule();
        } else {
          claw.Idle().schedule();
        }
      } else {
        state = States.GrabAlgaeOffReef; // Auto grab algae when ready
      }
    }
    manipulator.Off();
    climber.Off();
    // funnel.Down();
    funnel.Off();
  }

  private void drivetrainRequestHandler() {
    if (state == States.IntakeFromSource) { // Driver has controll of translation but not rotation
      drive
          .applyRequest(
              () ->
                  driveTrain
                      .withVelocityX(
                          MaxSpeed.times(
                              -driverLeft.getY())) // Drive forward with negative Y (forward)
                      .withVelocityY(MaxSpeed.times(-driverLeft.getX()))
                      .withRotationalRate(
                          Constants.MaxAngularRate.times(
                              (new Rotation2d(
                                          Units.degreesToRadians(
                                              TargetingComputer.getSourceTargetingAngle(
                                                  drive.getPose())))
                                      .minus(drive.getPose().getRotation())
                                      .getRadians())
                                  * rotP)))
          .schedule();
    } else if (state == States.GrabAlgaeOffReef
        || state == States.PlaceCoral
        || state == States.PrepAlgaeGrab
        || state
            == States
                .PrepCoralPlace) { // Driver has controll of nothing unless cameras haven't found
      // tag
      if (!vision.containsRequestedTarget(TargetingComputer.getCurrentTargetBranch().getApriltag())
          || Math.abs(
                  new Rotation2d(
                          Units.degreesToRadians(
                              TargetingComputer.getCurrentTargetBranch().getTargetingAngle()))
                      .minus(drive.getPose().getRotation())
                      .getDegrees())
              > 5) { // If the robot can NOT see the correct apriltag
        drive
            .applyRequest(
                () ->
                    driveTrain
                        .withVelocityX(
                            MaxSpeed.times(
                                -driverLeft.getY())) // Drive forward with negative Y (forward)
                        .withVelocityY(MaxSpeed.times(-driverLeft.getX()))
                        .withRotationalRate(
                            Constants.MaxAngularRate.times(
                                (new Rotation2d(
                                            Units.degreesToRadians(
                                                TargetingComputer.getCurrentTargetBranch()
                                                    .getTargetingAngle()))
                                        .minus(drive.getPose().getRotation())
                                        .getRadians())
                                    * rotP)))
            .schedule();
      } else if (vision.containsRequestedTarget(
              TargetingComputer.getCurrentTargetBranch().getApriltag())
          && Math.abs(
                  new Rotation2d(
                          Units.degreesToRadians(
                              TargetingComputer.getCurrentTargetBranch().getTargetingAngle()))
                      .minus(drive.getPose().getRotation())
                      .getDegrees())
              < 5) {
        drive
            .applyRequest(
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
                                        .minus(drive.getPose().getRotation())
                                        .getRadians())
                                    * rotP)))
            .schedule();
      }
    } else { // Driver has full controll
      drive
          .applyRequest(
              () ->
                  driveTrain
                      .withVelocityX(
                          MaxSpeed.times(
                              -driverLeft.getY())) // Drive forward with negative Y (forward)
                      .withVelocityY(
                          MaxSpeed.times(-driverLeft.getX())) // Drive left with negative X (left)
                      .withRotationalRate(
                          Constants.MaxAngularRate.times(
                              -driverRight
                                  .getX()))) // Drive counterclockwise with negative X (left)
          .schedule();
    }
  }
}
