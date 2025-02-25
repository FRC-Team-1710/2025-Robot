// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.TargetingComputer;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private static boolean redAlliance;
  Timer m_gcTimer = new Timer();

  public Robot() {
    redAlliance = checkRedAlliance();
    TargetingComputer.setAlliance(redAlliance);

    Pathfinding.setPathfinder(new LocalADStarAK());

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Output 1710 logo
    System.out.print(
        "\nF  I  R  S  T   R  O  B  O  T  I  C  S   T  E  A  M\n______________  _  _____   _  _____  ______________\n\\_____________|/ ||___  | / ||  _  ||_____________/\n \\_ _ _ _ _ _ || |   / /  | || | | || _ _ _ _ _ _/\n  \\ _ _ _ _ _ || |  / /   | || |_| || _ _ _ _ _ /\n   \\__________||_|_/_/___ |_||_____||__________/\n    \\___________________/ \\___________________/\n                     ___.^.___\n                     '.     .'\n                      /.' '.\\\n\n");

    // Lowers brownout threshold to 6.0V
    RobotController.setBrownoutVoltage(6.0);

    SignalLogger.stop();

    // Warmup the PPLib library
    FollowPathCommand.warmupCommand().schedule();
    // PathfindingCommand.warmupCommand().schedule();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 10);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putString(
        "Current Target", TargetingComputer.getCurrentTargetBranch().toString());
    Logger.recordOutput("Target Level", TargetingComputer.getCurrentTargetLevel().toString());
    SmartDashboard.putString(
        "Random Target Branch", TargetingComputer.getCurrentTargetForBranchGame().toString());
    SmartDashboard.putNumber("Branch Game Score", TargetingComputer.branchGameScore);
    // if (m_gcTimer.advanceIfElapsed(5)) System.gc();
    Logger.recordOutput("Time since startup", m_gcTimer.get());
  }

  /** Gets the current alliance, true is red */
  public static boolean getAlliance() {
    return redAlliance;
  }

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
    redAlliance = checkRedAlliance();
    TargetingComputer.setAlliance(redAlliance);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    redAlliance = checkRedAlliance();
    TargetingComputer.setAlliance(redAlliance);
    if (TargetingComputer.gameMode) {
      TargetingComputer.startBranchGame();
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (TargetingComputer.gameMode) {
      TargetingComputer.checkBranchGame();
    }
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
