package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorIOInputs;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The Elevator subsystem controls a dual-motor mechanism for game piece manipulation. It supports
 * multiple distances for different game actions
 */
@Logged
public class Elevator {
  // Hardware interface and inputs
  @Logged(name = "IO", importance = Importance.CRITICAL)
  private final ElevatorIO io;
  @Logged(name = "Inputs", importance = Importance.CRITICAL)
  private final ElevatorIOInputs inputs;

  // Current elevator distance mode
  @Logged(name = "CurrentState", importance = Importance.CRITICAL)
  private ElevatorStates currentState = ElevatorStates.INTAKE;

  @Logged(name = "Timer", importance = Importance.CRITICAL)
  private final Timer timer = new Timer();
  @NotLogged
  private boolean doneZeroing = false;

  @Logged(name = "CanMoveUp", importance = Importance.CRITICAL)
  private final BooleanSupplier canMoveUp;

  @Logged(name = "ManualOverride", importance = Importance.CRITICAL)
  private final DoubleSupplier manualSupplier;

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the elevator
   */
  public Elevator(ElevatorIO io, DoubleSupplier manualSupplier, BooleanSupplier canMoveUp) {
    this.io = io;
    this.inputs = new ElevatorIOInputs();
    this.manualSupplier = manualSupplier;
    this.canMoveUp = canMoveUp;
  }

  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);

    if (currentState != ElevatorStates.STOP && currentState != ElevatorStates.ZERO) {
      if (manualSupplier.getAsDouble() != 0) {
        io.setManual(manualSupplier.getAsDouble());
      } else {
        if (!inputs.locked) {
          // Resets pid controller
          io.setManual(0);
        }
        io.setDistance(currentState.targetDistance);
      }
    } else if (currentState == ElevatorStates.STOP) {
      io.stop();
    } else {
      if (doneZeroing) {
        io.zero();
        io.setManual(0);
      } else {
        doneZeroing = timer.hasElapsed(0.25) && inputs.leaderStatorCurrent.in(Amps) > 55;
        io.setManual(-0.1);
      }
    }
  }

  public enum ElevatorStates {
    STOP(Inches.of(0)), // Stop the elevator
    ZERO(Inches.of(0)), // Stop the elevator
    INTAKE(Inches.of(0), Inches.of(1.25)), // Elevator tucked in
    L1(Inches.of(0), Inches.of(1)), // Position for scoring in L1
    L2(Inches.of(15.75)), // Position for scoring in L2
    L3(Inches.of(32.25)), // Position for scoring in L3
    L4(Inches.of(54.5)), // Position for scoring in L4
    ALGAE_LOW(Inches.of(21), Inches.of(1.5)), // Position for grabbing low algae
    ALGAE_HIGH(Inches.of(35.5), Inches.of(1.5)); // Position for grabbing high algae

    private final Distance targetDistance;
    private final Distance angleTolerance;

    ElevatorStates(Distance targetDistance, Distance angleTolerance) {
      this.targetDistance = targetDistance;
      this.angleTolerance = angleTolerance;
    }

    ElevatorStates(Distance targetDistance) {
      this(targetDistance, Inches.of(1));
    }
  }

  @NotLogged
  public Distance getPosition() {
    return inputs.distance;
  }

  @NotLogged
  public ElevatorStates getState() {
    return currentState;
  }

  public void setState(ElevatorStates state) {
    if (state != ElevatorStates.ZERO) {
      timer.reset();
      doneZeroing = false;
    }
    if (!timer.isRunning()) {
      timer.start();
    }
    if (canMoveUp.getAsBoolean()) {
      this.currentState = state;
    } else {
      this.currentState = ElevatorStates.INTAKE;
    }
  }

  @NotLogged
  public boolean isDoneZeroing() {
    return doneZeroing;
  }

  @Logged(name = "IsAtTarget", importance = Importance.CRITICAL)
  public boolean isAtTarget() {
    if (currentState == ElevatorStates.STOP) return true;
    return getPosition()
        .isNear(
            currentState.targetDistance,
            Constants.currentMode == Mode.SIM ? Inches.of(2.5) : currentState.angleTolerance);
  }

  public void toggleKillSwitch() {
    inputs.killSwitch = inputs.killSwitch ? false : true;
  }
}
