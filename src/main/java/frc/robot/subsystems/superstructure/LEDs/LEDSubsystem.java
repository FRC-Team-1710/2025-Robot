// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Collin (King) Arthur
// Gavin (The Goat) Bigham   <--- Vs code autofilled The Goat for me :)

package frc.robot.subsystems.superstructure.LEDs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.TargetingComputer;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private SerialPort uart;

  Timer timer = new Timer();

  private Funnel funnel;
  private Manipulator manipulator;
  private Climber climber;
  private Elevator elevator;
  private Drive drivetrain;

  public LEDSubsystem(Funnel funnel, Manipulator manipulator, Climber climber, Elevator elevator, Drive drivetrain) {
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.climber = climber;
    this.elevator = elevator;
    uart = new SerialPort(115200, SerialPort.Port.kMXP); // Set baud rate
  }

  private Integer commandValue = 0;

  public Boolean[] inputBooleans = {
    false, false, false, false, false, false, false, false, false, false, false, false
  };

  // // Booleans to store the status of the beam breaks (Might not need these)
  // private Boolean
  //     firstManipulatorBreak; // Beam break to indicate if the coral has started to enter the
  // // manipulator
  // private Boolean
  //     secondManipulatorBreak; // Beam break to indicate if the coral has fully entered the
  // // manipulator
  // private Boolean
  //     firstFunnelBreak; // Ask about these next 2 beam breaks again (the ones in the funnel)
  // private Boolean secondFunnelBreak;

  @Override
  public void periodic() {
    set(); // Getting condition of robot
    encoder(); // Setting the command value
    sendData(commandValue); // Send Phase
  }

  /** Sets the input booleans based on the current state of the robot */
  private void set() { // Decimal phase

    if (DriverStation.getMatchTime() <= 25 && DriverStation.getMatchTime() > 20) {
      inputBooleans[0] = true;
    } else {
      inputBooleans[0] = false;
    }

    if (timer.get() >= 45 && timer.get() <= 50) {
      inputBooleans[1] = true;
    } else {
      inputBooleans[1] = false;
    }

// if (TargetingComputer.currentTargetLevel == TargetingComputer.Levels.L1) {
//       inputBooleans[3] = true;
//     } else {
//       inputBooleans[3] = false;
//   }
// I think that this and elevator.getMode do similar if not the same things? I'm not entirely sure

    if (elevator.getMode() == Elevator.ElevatorPosition.L1 && funnel.hasCoral()) {
      inputBooleans[2] = true;
    } else {
      inputBooleans[2] = false;
    }

    if (elevator.getMode() == Elevator.ElevatorPosition.L1) {
      inputBooleans[3] = true;
    } else {
      inputBooleans[3] = false;
    }

    // If the robot is in the alignment zone and the angle is within the tolerance (aligned)
    if (drivetrain.getDistanceToPose(TargetingComputer.getCurrentTargetBranchPose()).getNorm()
      < TargetingComputer.alignmentTranslationTolerance
    && elevator.isAtTarget()
    && manipulator.hasCoral()) {
    inputBooleans[4] = true; 
    } else {
      inputBooleans[4] = false;
    }

  // The drivetrain is in the alignment zone, the robot’s rotation is close to the target angle, the robot is close to where it should be (still aligning)
  if (drivetrain.isInAlignmentZone() 
    && Math.abs(
      new Rotation2d(
        Units.degreesToRadians(TargetingComputer.getCurrentTargetBranch().getTargetingAngle()))
        .minus(drivetrain.getPose().getRotation())
        .getDegrees())
      < TargetingComputer.alignmentAngleTolerance) {
    inputBooleans[5] = true;  
  } else {
    inputBooleans[5] = false;
  }

    inputBooleans[6] = climber.goForClimb;
    inputBooleans[7] = manipulator.hasCoral();
    inputBooleans[8] = funnel.hasCoral();

    if (Robot.checkRedAlliance()) {
      inputBooleans[9] = true;
      inputBooleans[10] = false;
    } else {
      inputBooleans[9] = false;
      inputBooleans[10] = true;
    }

    if (DriverStation.isDSAttached()) {
      inputBooleans[11] = true;
    } else {
      inputBooleans[11] = false;
    }


    SmartDashboard.putBooleanArray("Input Booleans", inputBooleans);
  }

  /**
   * Sets the input booleans to send based on the priorities of the states
   */
  private void encoder() { // Transition phase
    for (int i = 0;
        i < inputBooleans.length;
        i++) { // Picks the first true sequence based on priority
      if (inputBooleans[i]) {
        commandValue = i;
        break;
      }
    }
  }

  public void sendData(int value) { // Sending data (duh)
    byte[] data = new byte[1]; // Create a byte array of length 1
    data[0] =
        (byte) (value & 0xFF); // Store value as byte in the array, and mask to ensure unsigned byte
    uart.write(data, data.length); // Write the byte array to the serial port
    // System.out.println("Sending Data: " + value); // Print the data that we sent
  }
}
