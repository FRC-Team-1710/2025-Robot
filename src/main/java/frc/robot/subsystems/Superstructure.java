// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.LEDs.LEDSubsystem;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;

public class Superstructure extends SubsystemBase {
    private final Drive drivetrain;
    private final Claw claw;
    private final Climber climber;
    private final Elevator elevator;
    private final Funnel funnel;
    private final LEDSubsystem ledSubsystem;
    private final Manipulator manipulator;
    private final Vision vision;

    public enum WantedStateState {
        ZERO,
        STOPPED,
        DEFAULT_STATE,
        INTAKE_CORAL_FROM_STATION,
        SCORE_L1_MANUAL_ALIGN,
        SCORE_LEFT_L2,
        SCORE_LEFT_L3,
        SCORE_LEFT_L4,
        SCORE_RIGHT_L2,
        SCORE_RIGHT_L3,
        SCORE_RIGHT_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        INTAKE_ALGAE_FROM_REEF,
        INTAKE_ALGAE_FROM_GROUND,
        MOVE_ALGAE_TO_NET_POSITION,
        SCORE_ALGAE_IN_NET,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    public enum CurrentState {
        ZERO,
        STOPPED,
        NO_PIECE_TELEOP,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_CORAL_AUTO,
        HOLDING_ALGAE,
        INTAKE_CORAL_FROM_STATION,
        SCORE_TELEOP_L1_MANUAL_ALIGNMENT,
        SCORE_LEFT_TELEOP_L2,
        SCORE_LEFT_TELEOP_L3,
        SCORE_LEFT_TELEOP_L4,
        SCORE_RIGHT_TELEOP_L2,
        SCORE_RIGHT_TELEOP_L3,
        SCORE_RIGHT_TELEOP_L4,
        SCORE_AUTO_L1,
        SCORE_LEFT_AUTO_L2,
        SCORE_LEFT_AUTO_L3,
        SCORE_LEFT_AUTO_L4,
        SCORE_RIGHT_AUTO_L2,
        SCORE_RIGHT_AUTO_L3,
        SCORE_RIGHT_AUTO_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        INTAKE_ALGAE_FROM_HP,
        INTAKE_ALGAE_FROM_REEF,
        INTAKE_ALGAE_FROM_GROUND,
        INTAKE_ALGAE_FROM_MARK,
        MOVE_ALGAE_TO_NET_POSITION,
        SCORE_ALGAE_IN_NET,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    public Superstructure(Drive drivetrain,
            Claw claw,
            Climber climber,
            Elevator elevator,
            Funnel funnel,
            LEDSubsystem ledSubsystem,
            Manipulator manipulator,
            Vision vision) {
        this.drivetrain = drivetrain;
        this.claw = claw;
        this.climber = climber;
        this.elevator = elevator;
        this.funnel = funnel;
        this.ledSubsystem = ledSubsystem;
        this.manipulator = manipulator;
        this.vision = vision;
    }

    @Override
    public void periodic() {
    }
}
