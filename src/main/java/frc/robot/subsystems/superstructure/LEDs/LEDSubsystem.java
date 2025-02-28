// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.LEDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private SerialPort uart;

  public LEDSubsystem() {
      uart = new SerialPort(115200, SerialPort.Port.kMXP); // Set baud rate
  }

  private Boolean hasNote = false;
  private Boolean chargingOuttake = false;
  private Boolean atSpeed = false;

  private Integer commandValue = 0;

  @Override
  public void periodic() {
      set(); // Getting condition of robot
      encoder(); // Setting the command value
      sendData(commandValue); // Send Phase 
  }
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

  private void set() { // Decimal phase
      // Note Detected
      /*if (results.hasTargets()) {
          NoteDetected(true);
      } else {
          NoteDetected(false);
      }*/


      // Driver Station Connected
                                                        // Get Collin's help tomorrow to fix this
      if (DriverStation.isDSAttached()) {
          inputBooleans[0] = false;
      } else {
          inputBooleans[0] = true;
      }

      if (Robot.checkRedAlliance()) {
          inputBooleans[11] = true;
          inputBooleans[12] = false;
      } else {
          inputBooleans[11] = false;
          inputBooleans[12] = true;
      }

      if (DriverStation.getMatchTime() <= 30 && DriverStation.getMatchTime() > 27) {
          inputBooleans[4] = true;
      } else {
          inputBooleans[4] = false;
      }

      // HasNote for ChargingOuttake and AtSpeed
      if (hasNote) { // Converts from simple inputs to boolean
          if (chargingOuttake) {
              inputBooleans[5] = true;
              inputBooleans[6] = false;
          } else if (atSpeed) {
              inputBooleans[5] = false;
              inputBooleans[6] = true;
          }
          inputBooleans[7] = false;
          inputBooleans[8] = false;
      } else {
          if (chargingOuttake) {
              inputBooleans[7] = true;
              inputBooleans[8] = false;
          } else if (atSpeed) {
              inputBooleans[7] = false;
              inputBooleans[8] = true;
          }
          inputBooleans[5] = false;
          inputBooleans[6] = false;
      }

      /*
      // Check if pathfinding
      if (SwerveSubsystem.followingPath) {
          inputBooleans[0] = true;
      } else {
          inputBooleans[0] = false;
      }*/

      // Check beam breaks
      if (intexer.intakeBreak()) {
          inputBooleans[9] = true; // Intake
          inputBooleans[10] = false; // Shooter
          hasNote = true;
      } else if (intexer.shooterBreak()) {
          inputBooleans[9] = false; // Intake
          inputBooleans[10] = true; // Shooter
          hasNote = true;
      } else {
          inputBooleans[9] = false; // Intake
          inputBooleans[10] = false; // Shooter
          hasNote = false;
      }

      // Charging or At Speed with Note or without Note
      if (hasNote) { // Has Note
          if (shooter.isShooterAtSpeed()) { // At speed
              inputBooleans[5] = false;
              inputBooleans[6] = true;
          } else if (shooter.getVelocity() > Constants.Shooter.idleSpeedRPM + 500) { // Charging
              inputBooleans[5] = true;
              inputBooleans[6] = false;
          } else {
              inputBooleans[5] = false;
              inputBooleans[6] = false;
          }
          inputBooleans[7] = false;
          inputBooleans[8] = false;
      } else { // No Note
          if (shooter.isShooterAtSpeed()) { // At speed
              inputBooleans[7] = false;
              inputBooleans[8] = true;
          } else if (shooter.getVelocity() > Constants.Shooter.idleSpeedRPM + 500) { // Charging
              inputBooleans[7] = true;
              inputBooleans[8] = false;
          } else {
              inputBooleans[7] = false;
              inputBooleans[8] = false;
          }
          inputBooleans[5] = false;
          inputBooleans[6] = false;
      }

      SmartDashboard.putBooleanArray("Input Booleans", inputBooleans);
  }

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

  public void sendData(int value) {
      byte[] data = new byte[1]; // Create a byte array of length 1
      data[0] = (byte) (value & 0xFF); // Store value as byte in the array, and mask to ensure unsigned byte
      uart.write(data, data.length); // Write the byte array to the serial port
      //System.out.println("Sending Data: " + value); // Print the data that we sent
  }
}


