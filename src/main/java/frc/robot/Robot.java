// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.SimCoral;
import java.util.Optional;

@Logged
public class Robot extends TimedRobot {
  @Logged(name = "AutonomousCommand", importance = Importance.DEBUG)
  private Command m_autonomousCommand;

  // Configuration constants
  @Logged(name = "BeforeMatch", importance = Importance.DEBUG)
  public static volatile boolean BEFORE_MATCH = true; // Controls MT1-only usage before match

  @Logged(name = "RobotContainer", importance = Importance.CRITICAL)
  private final RobotContainer m_robotContainer;

  @Logged(name = "RedAlliance", importance = Importance.DEBUG)
  private static boolean redAlliance;

  Timer m_gcTimer = new Timer();

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);

    Epilogue.configure(
        config -> {
          if (isSimulation()) {
            config.errorHandler = ErrorHandler.crashOnError();
          } else {
            config.errorHandler = ErrorHandler.printErrorMessages();
          }

          config.root = "Telemetry";

          config.minimumImportance = Constants.importance;
        });

    Epilogue.bind(this);

    DataLogManager.start();

    redAlliance = checkRedAlliance();

    // Output 1710 logo
    System.out.print(
        "\nF  I  R  S  T   R  O  B  O  T  I  C  S   T  E  A  M\n______________  _  _____   _  _____  ______________\n\\_____________|/ ||___  | / ||  _  ||_____________/\n \\_ _ _ _ _ _ || |   / /  | || | | || _ _ _ _ _ _/\n  \\ _ _ _ _ _ || |  / /   | || |_| || _ _ _ _ _ /\n   \\__________||_|_/_/___ |_||_____||__________/\n    \\___________________/ \\___________________/\n                     ___.^.___\n                     '.     .'\n                      /.' '.\\\n\n");

    // Lowers brownout threshold to 6.0V
    RobotController.setBrownoutVoltage(6.0);

    DriverStation.silenceJoystickConnectionWarning(true);

    SignalLogger.setPath("/U/ctre-logs");
    SignalLogger.stop();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();

    m_robotContainer.setAlliance(redAlliance);

    SimCoral.setRedAlliance(redAlliance);

    m_gcTimer.start();

    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  /** Gets the current alliance, true is red */
  @NotLogged
  public static boolean getAlliance() {
    return redAlliance;
  }

  @NotLogged
  public static boolean checkRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      DataLogManager.log("ERROR: Alliance not found. Defaulting to Blue");
      return false;
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    BEFORE_MATCH = false;
    redAlliance = checkRedAlliance();
    m_robotContainer.setAlliance(redAlliance);

    SimCoral.setRedAlliance(redAlliance);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_robotContainer.requestDefault();
    }
  }

  @Override
  public void teleopInit() {
    BEFORE_MATCH = false;
    redAlliance = checkRedAlliance();
    m_robotContainer.setAlliance(redAlliance);

    SimCoral.setRedAlliance(redAlliance);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_robotContainer.requestDefault();
    }

    SignalLogger.start();
  }

  @Override
  public void teleopPeriodic() {
    SignalLogger.stop();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
