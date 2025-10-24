// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.SimCoral;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  // Configuration constants
  public static volatile boolean BEFORE_MATCH = true; // Controls MT1-only usage before match

  private final RobotContainer m_robotContainer;

  private static boolean redAlliance;
  Timer m_gcTimer = new Timer();

  public Robot() {
    redAlliance = checkRedAlliance();

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

    // Senior Prank
    System.out.print("                       &&&&&&&&&&&&&&&&                            \n                 &$X$&&&&&$$$X$&&&$$x$$&&&&&&                      \n             &&Xxx++;++x+++++;;XX$x+;;xXX&&$&&&&&                  \n          &&$$xXxxxxX+:::.::+x+;:;::...+++x$X+X$$&&&               \n        &$XXX$$XX++x+:;+:;x;+:::.:.:..;;;x+;+$$XXXX$&&             \n       &XX$$$XXXXx++xx+xx+;::::::;::.:;::++;+x+++x++;x$&           \n      &XXXXX$Xxx$x+XX+;;;+x+;:;;..:+;;;..::::+++x++;+;x$&&         \n     &$X$XX+x+xXx;;:..:;x;+++;+;+;;;;:;.;::;.;;;+;;+;;++x$&        \n    &$X;++;++;;;x;+XXXx+x;xX;+xx::;:::.;x;;.:;;+:;:;x+.;++x$       \n   &X;;;+;;;++X$XXXXxx+x+x+xXx;.:;;::;;;;::::.;;.;:;;+x+;;++X&     \n   x::;+:::::::..;;;;;;;:xxx;.:;+;;x+x+;+:+::.;+;:;:;:;++++++x&    \n  X:++;:...:.....:;;;:::.;;.:++X;;++++;;+;;::x;+;:+:;;+;+xxx++$    \n  $;;;...........:.::::;:;;;+XxX$;;+:++++:;;+X;::;+++;+x++x;+x+$   \n $;;..:.....:::;:;++x++xxXxxxxXx+:+;;;+++++X+;;:;;;+xx+x+x+xxx+$   \n&+;:...:..:;;;+x+xXXXXX$$XXXxxx;;++++XxxxxXx+;;;;;;;+x+X++xxxxXX&  \n$+:.....:;+xxxxXXXX$$$X$$XX$XX++;+++xXX$Xxxx++x+x;:;+;+x++xxxxX$&  \n&;..:::;xX$XXXXX$$XXXXXX$$$XXxXXxXXXXXX$$Xxxxx+++;++++;++xXxxx$&&  \n +...:+x$XXXX$$$$$$XXXXX$$$XX$$$$$$&$$&$$XXxxxxx+;+xx+xXxX$XX$X$X  \n  +.::x$&&&&&&&&&&&$XXXXXX$$$$$&$$$&$$$$$XXxxxxxxxXXXXX$xXxx++xXx& \n   ::;X&&&$$$$$X$$$&&&&&&$$$$$$$&$$$&&&&$XXXx$$$X$$X$$$$$$XXx;x++$ \n   +:+$&&&&&&&&&&&&&&$$&&&&$&$&&&$&&&&&$$$$$$$&&$&&&&&&&&&$$X;;;+$ \n   $:x&&&&&&&&&&&&&&&&&&&&&&&&&$&$&&$$&&&&&&&&&&&&&&&&&&&&&&&$++;& \n  &$+$&&&$XxxxxX$&&&&&&&&&&&&&&&&&&&$&$&&&&&$&&&&&&&&&&&&&&&&X+;+  \n  &+x$&&X+;:;::;;;XXX$$&&&&&&&&&&&&&&&$$&&&&&&&&&&&&&&&&&&&&&$;:x  \n   X;X&&$xxxx++;:..:;+xXX$$$$$$$&&$&&&&&&&&&&&&&&&&&&&&&&&&&&x:X   \n   $+X$&Xxxxxx++x++;:.;;xxXxX$$$X$$$$$&&&&&&&&&&&&&&&&&&&&&&&++&   \n   $x&$$Xx+++;;::.::::;:::;+xxXXXX$$$$$$$$&&&&&&&&&&&&&&&&&&&+x    \n  X;&&$$$x++;:+;......::::::++xxxxxXXXX$$$$&$$$&$$$&&&&&&&&&&;x    \n XX$$$XXXXx;;;::;x:.....::.:;;+++xxxxxxxXXXXxxxx+;+x+xXXXXX$&x+X   \n :    ..............:::++::;:;;++xxxx++;:::::::;:;::;;+;x+X$$&$+   \n  ;   x&$&&&$XXX+;+x;;.:....;:;+XX$$$Xx:.......  .:..:;+XX$&&&&xx  \n   ; .x&$&&&&&Xxx++;;:;+:;.;.   .........:... ..........::.;X&&&;x \n    x.:$&$&&&&&$x++;;+;;;;;++;          .;::::;;;+;+xXXX$XX;      .\n     .;x$$$$&&&$&Xxxx++;xx++;;  :;+x:  xXx;::::;;;++X$$$$&&; ;   . \n     &.:$$$X$$&&$$$$XXxxx+++;. .;x$$X. XXXX+;:;++xxX$$$$&&$    .+  \n      &.X$$$XX$$$$&$XXx++;xx. .;xX$$$; xXXxx++;++xXxxX$$&&+:.; :   \n       &.;$XXX$X$$$XXxx++xx. .xXXXX$$X..xxxxxXxx++x+xX$$$x  . +    \n        $X;.:x$XXxxxxx;;;:. :XX$$$$$$$X .;+xxxxXxx+xXX$$x  :.X     \n        $XXxxxx+;:::...:::$XX$&$$$$$$$Xx. .:;+;xxxxxXX$x.;. &      \n        &Xxxxxx+++++;;;++XXX$$&&&$$$&&$XX;:...:;XXX$Xx;. x         \n         Xxx+++;+++;;+xxx+;+X$$$$$$&$&$$X+:::;;++xxXX$$&&          \n          x+;;;;;;;++xxXXx+::+xXXXXXXXxx++;;;;;++xXX$$$&           \n           x+;;;;;;;++xXXXX;::::::;+xX$XXXXxxxxxXXXX$$&            \n            x;;;+;;;::;xXXx+;::::;xXXX$XXXXXXXXXXX$$X$&            \n             X+;;;::::::xxxx++xxxxX$$$$$XxxxXX$XXX$X$&             \n              $;;;;;;;;;::::::+xx+xXX$$X+::+xX$$$$X$$              \n                X;::::;;x++;:::::::::;+xxX+++xXXXX$                \n                 &;:::::::;;;;;;;++++xXxxx++++xxX                  \n                   +;;++;;::::::::::::;;;;;+++x                    \n                    $++xxx++;;;;+xx++++xx+++x                      \n                      ++xXxxx+xxxX$XXXXXx+$                        \n                       X;++++++++xXXXxx+                           \n                          X+;::;;;;++&                             \n     ____ ___ ____   ____  ____   ___ _____ _   _ _____ ____    \n    | __ )_ _/ ___| | __ )|  _ \\ / _ \\_   _| | | | ____|  _ \\   \n    |  _ \\| | |  _  |  _ \\| |_) | | | || | | |_| |  _| | |_) |  \n    | |_) | | |_| | | |_) |  _ <| |_| || | |  _  | |___|  _ <   \n    |____/___\\____| |____/|_| \\_\\\\___/_|_|_|_|_|_|_____|_| \\_\\_ \n    |_ _/ ___|  \\ \\      / / \\|_   _/ ___| | | |_ _| \\ | |/ ___|\n     | |\\___ \\   \\ \\ /\\ / / _ \\ | || |   | |_| || ||  \\| | |  _ \n     | | ___) |   \\ V  V / ___ \\| || |___|  _  || || |\\  | |_| |\n    |___|____/     \\_/\\_/_/   \\_\\_| \\____|_| |_|___|_| \\_|\\____|\n");

    // Lowers brownout threshold to 6.0V
    RobotController.setBrownoutVoltage(6.0);

    DriverStation.silenceJoystickConnectionWarning(true);

    SignalLogger.stop();

    // Set Pathfinding to the default AdvantageKit Pathfinder

    Pathfinding.setPathfinder(new LocalADStarAK());

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    // Warmup the PPLib library

    m_robotContainer = new RobotContainer();

    m_robotContainer.setAlliance(redAlliance);

    SimCoral.setRedAlliance(redAlliance);

    // Logging for watching logs
    Logger.recordOutput(
        "Control Mode",
        Constants.babyControlMode ? "Bummy baby controls" : "Full speed straight into the reef");

    m_gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 10);
    Logger.recordOutput("Match Time", DriverStation.getMatchTime());
    Logger.recordOutput("Time since startup", m_gcTimer.get());
    m_robotContainer.autoPeriodic();
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
      m_robotContainer.requsetDefault();
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
      m_robotContainer.requsetDefault();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
