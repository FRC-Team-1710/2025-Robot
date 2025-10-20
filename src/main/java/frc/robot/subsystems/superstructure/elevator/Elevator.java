package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
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
  private ElevatorStates currentState = ElevatorStates.INTAKE;

  private final Timer timer = new Timer();
  private boolean doneZeroing = false;

  private final DoubleSupplier manualSupplier;

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the elevator
   */
  public Elevator(ElevatorIO io, DoubleSupplier manualSupplier) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    this.manualSupplier = manualSupplier;
  }

  @Override
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

    Logger.processInputs("Elevator", inputs);
  }

  public enum ElevatorStates {
    STOP(Inches.of(0)), // Stop the elevator
    ZERO(Inches.of(0)), // Stop the elevator
    INTAKE(Inches.of(0), Inches.of(1.25)), // Elevator tucked in
    L1(Inches.of(0), Inches.of(1)), // Position for scoring in L1
    L2(Inches.of(15.75)), // Position for scoring in L2
    L3(Inches.of(32.25)), // Position for scoring in L3
    L4(Inches.of(54.5)), // Position for scoring in L4
    ALGAE_LOW(Inches.of(19), Inches.of(1.5)), // Position for grabbing low algae
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

  @AutoLogOutput
  public Distance getPosition() {
    return inputs.distance;
  }

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
    this.currentState = state;
  }

  public boolean isDoneZeroing() {
    return doneZeroing;
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentState == ElevatorStates.STOP) return true;
    return getPosition().isNear(currentState.targetDistance, currentState.angleTolerance);
  }

  @AutoLogOutput
  public void toggleKillSwich() {
    inputs.killSwich = inputs.killSwich ? false : true;
  }
}
