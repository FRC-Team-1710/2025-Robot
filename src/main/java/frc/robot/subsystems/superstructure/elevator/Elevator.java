package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Conversions;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls a dual-motor mechanism for game piece manipulation. It supports
 * multiple distances for different game actions
 */
public class Elevator extends SubsystemBase {
  // Hardware interface and inputs
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  // Current elevator distance mode
  private ElevatorPosition currentMode = ElevatorPosition.INTAKE;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Elevator leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Elevator follower motor isn't connected", AlertType.kError);

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the elevator
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    Logger.recordOutput(
        "motor encoder dist",
        Conversions.rotationsToDistance(inputs.leaderRotorPosition, 6, Inches.of(1.105)));
  }

  /**
   * Runs the elevator in distance mode to the specified distance.
   *
   * @param distance The target angle distance
   */
  public void setDistance(Distance distance) {
    io.setDistance(distance);
  }

  /**
   * @param power power between -1 & 1
   */
  public void setManual(double power) {
    io.setManual(power);
  }

  public void stopHere() {
    io.stopHere();
  }

  /** Stops the elevator motors. */
  private void stop() {
    io.stop();
  }

  /** Sets the current potentiometer position to zero */
  public void zero() {
    io.zero();
  }

  public double getElevatorCurrent() {
    return inputs.leaderStatorCurrent.baseUnitMagnitude();
  }

  /**
   * Returns the current distance of the elevator.
   *
   * @return The current distance (Inches)
   */
  @AutoLogOutput
  public Distance getPosition() {
    return inputs.elevatorDistance;
  }

  /** Enumeration of available elevator distances with their corresponding target angles. */
  public enum ElevatorPosition {
    STOP(Inches.of(0)), // Stop the elevator
    INTAKE(Inches.of(0), Inches.of(1.25)), // Elevator tucked in
    L1(Inches.of(0), Inches.of(1)), // Position for scoring in L1
    L2(Inches.of(15.75)), // Position for scoring in L2
    L3(Inches.of(32.25)), // Position for scoring in L3
    L4(Inches.of(54.5)), // Position for scoring in L4
    ALGAE_LOW(Inches.of(19), Inches.of(1.5)), // Position for grabbing low algae
    ALGAE_HIGH(Inches.of(35.5), Inches.of(1.5)); // Position for grabbing high algae

    private final Distance targetDistance;
    private final Distance angleTolerance;

    ElevatorPosition(Distance targetDistance, Distance angleTolerance) {
      this.targetDistance = targetDistance;
      this.angleTolerance = angleTolerance;
    }

    ElevatorPosition(Distance targetDistance) {
      this(targetDistance, Inches.of(1)); // 2 inch default tolerance
    }
  }

  /**
   * Gets the current elevator distance mode.
   *
   * @return The current ElevatorPosition
   */
  public ElevatorPosition getMode() {
    return currentMode;
  }

  /**
   * @return if elevator is ready to intake
   */
  public boolean isAtIntake() {
    return isAtTarget() && getMode() == ElevatorPosition.INTAKE;
  }

  /**
   * Sets a new elevator distance and schedules the corresponding command.
   *
   * @param mode The desired ElevatorPosition
   */
  public void setElevatorPosition(ElevatorPosition mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current distance
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ElevatorPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Elevator"),
              ElevatorPosition.INTAKE,
              createPositionCommand(ElevatorPosition.INTAKE),
              ElevatorPosition.L1,
              createPositionCommand(ElevatorPosition.L1),
              ElevatorPosition.L2,
              createPositionCommand(ElevatorPosition.L2),
              ElevatorPosition.L3,
              createPositionCommand(ElevatorPosition.L3),
              ElevatorPosition.L4,
              createPositionCommand(ElevatorPosition.L4),
              ElevatorPosition.ALGAE_LOW,
              createPositionCommand(ElevatorPosition.ALGAE_LOW),
              ElevatorPosition.ALGAE_HIGH,
              createPositionCommand(ElevatorPosition.ALGAE_HIGH)),
          this::getMode);

  /**
   * Creates a command for a specific arm distance that moves the arm and checks the target
   * distance.
   *
   * @param distance The arm distance to create a command for
   * @return A command that implements the arm movement
   */
  public Command createPositionCommand(ElevatorPosition distance) {
    return Commands.runOnce(() -> setDistance(distance.targetDistance))
        .withName("Move to " + distance.toString());
  }

  /**
   * Checks if the arm is at its target distance.
   *
   * @return true if at target distance, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ElevatorPosition.STOP) return true;
    return getPosition().isNear(currentMode.targetDistance, currentMode.angleTolerance);
  }

  public boolean goingToTarget() {

    return getPosition().isNear(Inches.of(43), currentMode.angleTolerance);
  }

  /** Only allows manual inputs because it wants to break itself */
  @AutoLogOutput
  public void toggleKillSwich() {
    inputs.killSwich = inputs.killSwich ? false : true;
  }

  @AutoLogOutput
  public boolean isClearOfStage1() {
    return inputs.elevatorDistance.in(Inches) > 30;
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  private Distance targetDistance() {
    return currentMode.targetDistance;
  }

  /**
   * Creates a command to set the arm to a specific distance.
   *
   * @param distance The desired arm distance
   * @return Command to set the distance
   */
  private Command setPositionCommand(ElevatorPosition distance) {
    return Commands.runOnce(() -> setElevatorPosition(distance))
        .withName("SetElevatorPosition(" + distance.toString() + ")");
  }

  /** Factory methods for common distance commands */

  /**
   * @return Command to move the arm to L1 scoring distance
   */
  public final Command L1() {
    return setPositionCommand(ElevatorPosition.L1);
  }

  /**
   * @return Command to move the arm to L2 scoring distance
   */
  public final Command L2() {
    return setPositionCommand(ElevatorPosition.L2)
        .until(() -> ElevatorPosition.L2.targetDistance == getPosition());
  }

  /**
   * @return Command to move the arm to L3 distance
   */
  public final Command L3() {
    return setPositionCommand(ElevatorPosition.L3);
  }

  /**
   * @return Command to move the arm to L4 distance
   */
  public final Command L4() {
    return setPositionCommand(ElevatorPosition.L4);
  }

  /**
   * @return Command to move the arm to the low algae distance
   */
  public final Command ALGAE_LOW() {
    return setPositionCommand(ElevatorPosition.ALGAE_LOW);
  }

  /**
   * @return Command to move the arm to the high algae distance
   */
  public final Command ALGAE_HIGH() {
    return setPositionCommand(ElevatorPosition.ALGAE_HIGH);
  }

  /**
   * @return Command to intake the arm
   */
  public final Command INTAKE() {
    return setPositionCommand(ElevatorPosition.INTAKE)
        .until(() -> ElevatorPosition.L2.targetDistance == getPosition());
  }

  /**
   * @return Command to stop the arm
   */
  public final Command stopCommand() {
    return setPositionCommand(ElevatorPosition.STOP);
  }
}
