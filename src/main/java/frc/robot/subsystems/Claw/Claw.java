// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.claw;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls a dual-motor arm mechanism for game piece manipulation. It
 * supports multiple distances for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Claw extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs;

  private TalonFX clawRollers;
  private TalonFX wrist;
  private CANcoder ClawEncoder;

  private double CanEncoderOffset = 0;
  private double setPoint = 0;
  private double ClawGearing = 9 * (48 / 18);

  // PID
  private PIDController PIDWrist;
  private double VelocityP = 0;
  private double VelocityI = 0;
  private double VelocityD = 0;

  /**
   * Creates a new Elevator subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Claw(ClawIO io) {
    this.io = io;
    this.inputs = new ClawIOInputsAutoLogged();

    clawRollers = new TalonFX(0);
    wrist = new TalonFX(0);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    clawRollers.getConfigurator().apply(config);
    wrist.getConfigurator().apply(config);

    PIDWrist = new PIDController(VelocityP, VelocityI, VelocityD);

    wrist.setPosition(wrist.getPosition().getValueAsDouble() - CanEncoderOffset);
  }

  public void setClawPower(double power) {
    clawRollers.set(power);
  }

  public double getPosition() {
    return clawRollers.getPosition().getValueAsDouble() / ClawGearing;
  }

  public void setWrist(double position) {
    setPoint = position * ClawGearing;
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);

    io.updatePID(inputs);
  }

  /**
   * Runs the claw in closed-loop position mode to the specified angle.
   *
   * @param distance The target angle (degrees)
   */
  public void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  /**
   * Current position of claw
   *
   * @return Current angle in degrees
   */
  public Angle getAngle() {
    return inputs.angle;
  }

  /**
   * Set the claw angular speed manualy. WARNING: WILL OVERRIDE THE SET ANGLE!!!
   *
   * @param double Power from 1 to -1
   */
  public void setManual(double power) {
    io.setManual(power);
  }

  /**
   * Set the claw intake speed manualy.
   *
   * @param double Power from 1 to -1
   */
  public void runPercent(double power) {
    io.runPercent(power);
  }
}
