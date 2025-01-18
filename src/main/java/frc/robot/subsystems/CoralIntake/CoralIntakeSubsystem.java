// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.CoralIntake;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class CoralIntakeSubsystem extends SubsystemBase {
//   // Motor
//   private TalonFX ClawMotor;

//   // Breaking Beam
//   private DigitalInput breakingBeam;

//   public void ClawSubsystem() {
//     ClawMotor = new TalonFX(0);
//     breakingBeam = new DigitalInput(0);

//     // Falcon Setup
//     TalonFXConfiguration clawConfig = new TalonFXConfiguration();
//     clawConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//     clawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//     ClawMotor.getConfigurator().apply(clawConfig); // Apply Configurations

//     // SmartDashboard
//     SmartDashboard.putNumber("Claw/Claw Power", 0);
//     SmartDashboard.putBoolean("Claw/Beam Broken?", false);
//   }

//   @Override
//   public void periodic() {}

//   /*Power*/
//   public void setClawPower(double power) {
//     ClawMotor.set(power);
//     SmartDashboard.putNumber("Claw/Claw Power", power);
//   }

//   /*Boolean*/
//   public boolean beamBroken() {
//     SmartDashboard.putBoolean("Claw/Beam Broken?", !breakingBeam.get());
//     return !breakingBeam.get();
//   }
// }
