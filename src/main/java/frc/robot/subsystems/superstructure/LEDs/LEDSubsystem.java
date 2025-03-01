// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Collin (King) Arthur
// Gavin (The Goat) Bigham   <--- Vs code autofilled The Goat for me :)

package frc.robot.subsystems.superstructure.LEDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private SerialPort uart;
  Timer timer = new Timer();

    private Funnel funnel;
    private Manipulator manipulator;
    private Climber climber;
    private Elevator elevator;

public LEDSubsystem(Funnel funnel, Manipulator manipulator, Climber climber) {
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.climber = climber;
    this.elevator = elevator;
    uart = new SerialPort(115200, SerialPort.Port.kMXP); // Set baud rate
}

  private Integer commandValue = 0;

  public Boolean[] inputBooleans = {
    false, 
    false, 
    false,
    false,
    false, 
    false,
    false,
    false, 
    false, 
    false, 
    false, 
    false, 
    false 
};

// Booleans to store the status of the beam breaks (Might not need these)
private Boolean firstManipulatorBreak; // Beam break to indicate if the coral has started to enter the manipulator
private Boolean secondManipulatorBreak; // Beam break to indicate if the coral has fully entered the manipulator
private Boolean firstFunnelBreak; // Ask about these next 2 beam breaks again (the ones in the funnel)
private Boolean secondFunnelBreak; 

  @Override
  public void periodic() {
      set(); // Getting condition of robot
      encoder(); // Setting the command value
      sendData(commandValue); // Send Phase 
  }

  /**
   * Sets the input booleans based on the current state of the robot
   */
  private void set() { // Decimal phase
      
    if (DriverStation.isDSAttached()) {
        inputBooleans[0] = true;
        inputBooleans[1] = false;
        } else {
        inputBooleans[0] = false;
        inputBooleans[1] = true;
    }

    if (Robot.checkRedAlliance()) {
        inputBooleans[11] = true;
        inputBooleans[12] = false;
    } else {
        inputBooleans[11] = false;
        inputBooleans[12] = true;
    }

    if (DriverStation.getMatchTime() <= 25 && DriverStation.getMatchTime() > 20) {
        inputBooleans[4] = true;
    } else {
        inputBooleans[4] = false;
    }

    if (timer.get() >= 45 && timer.get() <= 50) {
        inputBooleans[5] = true;
    } else {
        inputBooleans[5] = false;
    }

    // if (firstManipulatorBreak) {
    //     inputBooleans[6] = true;
    // } else {
    //     inputBooleans[6] = false;
    // }

    inputBooleans[7] = funnel.hasCoral();
    inputBooleans[8] = manipulator.hasCoral();
    inputBooleans[9] = climber.goForClimb;

    if (elevator.getMode() == Elevator.ElevatorPosition.L1) {
        inputBooleans[10] = true;
    } else {
        inputBooleans[10] = false;
    }

    if (elevator.getMode() == Elevator.ElevatorPosition.L1 && funnel.hasCoral()) {
        inputBooleans[2] = true;
    } else {
        inputBooleans[2] = false;
    }


    SmartDashboard.putBooleanArray("Input Booleans", inputBooleans);
  }

  /**
   * Sets the input booleans to send based on the priorities of the states 
   * (Ex. Robot starts flying -> 0011)
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
      data[0] = (byte) (value & 0xFF); // Store value as byte in the array, and mask to ensure unsigned byte
      uart.write(data, data.length); // Write the byte array to the serial port
      //System.out.println("Sending Data: " + value); // Print the data that we sent
  }
}

