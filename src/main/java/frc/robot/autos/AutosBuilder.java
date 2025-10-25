// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;
import java.util.ArrayList;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class AutosBuilder {
  private final Superstructure superstructure;

  private ArrayList<Command> commandList = new ArrayList<>();

  private NextCommand nextCommand;
  private Source source;
  private SourceDistance sourceDistance;
  private Reef reef;
  private ReefHeight reefHeight;

  LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  private String customString = "";

  private Command preBuiltAuto = Commands.none();

  HashMap<Character, Source> charToSource = new HashMap<Character, Source>();
  HashMap<Character, Reef> charToReef = new HashMap<Character, Reef>();
  HashMap<Character, ReefHeight> charToReefHeight = new HashMap<Character, ReefHeight>();
  HashMap<Character, SourceDistance> charToSourceDistance =
      new HashMap<Character, SourceDistance>();

  public AutosBuilder(Superstructure superstructure) {
    charToSource.put('R', Source.RIGHT);
    charToSource.put('N', Source.LEFT);
    charToReef.put('A', Reef.A);
    charToReef.put('B', Reef.B);
    charToReef.put('C', Reef.C);
    charToReef.put('D', Reef.D);
    charToReef.put('E', Reef.E);
    charToReef.put('F', Reef.F);
    charToReef.put('G', Reef.G);
    charToReef.put('H', Reef.H);
    charToReef.put('I', Reef.I);
    charToReef.put('J', Reef.J);
    charToReef.put('K', Reef.K);
    charToReef.put('L', Reef.L);
    charToReefHeight.put('2', ReefHeight.L2);
    charToReefHeight.put('3', ReefHeight.L3);
    charToReefHeight.put('4', ReefHeight.L4);
    charToSourceDistance.put('F', SourceDistance.FAR);
    charToSourceDistance.put('M', SourceDistance.MID);
    charToSourceDistance.put('C', SourceDistance.CLOSE);

    this.superstructure = superstructure;
    SmartDashboard.putString("Custom Auto Input", "(insert auto here)");
    SmartDashboard.putString(
        "Custom Auto Input Key", "(A-L=Pipe,2-4=Level),(RN=RightOrLeftSource,FMC=FarOrMidOrCloes)");
    autoChooser.addDefaultOption("IDLE", Auto.IDLE);
    autoChooser.addDefaultOption("E4RFD4RFC4RMB4", Auto.E4RFD4RFC4RMB4);
    autoChooser.addDefaultOption("J4NFK4NFL4NMA4", Auto.J4NFK4NFL4NMA4);
    autoChooser.addOption("CUSTOM", Auto.CUSTOM);

    Logger.recordOutput("AutosBuilder/CommandList", commandList.toString());
  }

  public void periodic() {
    if (autoChooser.get() == Auto.CUSTOM
        && customString != SmartDashboard.getString("Custom Auto Input", "(insert auto here)")) {
      customString = SmartDashboard.getString("Custom Auto Input", "(insert auto here)");
      String output = validateAuto(customString);
      Logger.recordOutput("Is Auto Valid", output == "");
      Logger.recordOutput("Auto Validation Error", output);
      preBuiltAuto = buildAuto();
    } else if (autoChooser.get() != Auto.CUSTOM) {
      Logger.recordOutput("Is Auto Valid", true);
      Logger.recordOutput("Auto Validation Error", "");
    }
  }

  public String validateAuto(String input) {
    boolean first = true;
    for (int i = 0; i < input.length(); i++) {
      char character = input.charAt(i);
      if (first) {
        if (charToSource.containsKey(character)) {
          nextCommand = NextCommand.SOURCE;
          source = charToSource.get(character);
        } else if (charToReef.containsKey(character)) {
          nextCommand = NextCommand.PLACE;
          reef = charToReef.get(character);
        } else {
          return "Character at character "
              + (i + 1)
              + " of the first half was "
              + String.valueOf(character)
              + " which is invalid";
        }
      } else {
        if (nextCommand == NextCommand.PLACE) {
          if (charToReefHeight.containsKey(character)) {
            reefHeight = charToReefHeight.get(character);
          } else {
            return "Character at character "
                + (i + 1)
                + " of the second half was "
                + String.valueOf(character)
                + " while trying to "
                + nextCommand.toString()
                + " which is invalid";
          }
        } else {
          if (charToSourceDistance.containsKey(character)) {
            sourceDistance = charToSourceDistance.get(character);
          } else {
            return "Character at character "
                + (i + 1)
                + " of the second half was "
                + String.valueOf(character)
                + " while trying to "
                + nextCommand.toString()
                + " which is invalid";
          }
        }
      }
      first = !first;
    }
    return "";
  }

  public Command getAuto() {
    return autoChooser.get() == Auto.CUSTOM ? preBuiltAuto : buildAuto();
  }

  public Command buildAuto() {
    commandList = new ArrayList<>();
    if (Constants.currentMode == Mode.SIM) {
      commandList.add(Commands.runOnce(() -> superstructure.beginSimAuto()));
    }
    switch (autoChooser.get()) {
      case CUSTOM:
        if (SmartDashboard.getString("Custom Auto Input", "(insert auto here)")
            == "(insert auto here)") {
          return Commands.runOnce(() -> superstructure.setWantedState(WantedState.ZERO));
        } else {
          return buildAuto(SmartDashboard.getString("Custom Auto Input", "(insert auto here)"));
        }
      case IDLE:
        return Commands.runOnce(() -> superstructure.setWantedState(WantedState.ZERO));
      default:
        return buildAuto(autoChooser.get().toString());
    }
  }

  private Command buildAuto(String input) {
    boolean first = true;
    for (int i = 0; i < input.length(); i++) {
      char character = input.charAt(i);
      if (first) {
        if (charToSource.containsKey(character)) {
          nextCommand = NextCommand.SOURCE;
          source = charToSource.get(character);
        } else if (charToReef.containsKey(character)) {
          nextCommand = NextCommand.PLACE;
          reef = charToReef.get(character);
        } else {
          System.out.println("First half command wasn't real");
        }
      } else {
        if (charToReefHeight.containsKey(character)) {
          reefHeight = charToReefHeight.get(character);
        } else if (charToSourceDistance.containsKey(character)) {
          sourceDistance = charToSourceDistance.get(character);
        }
        commandList.add(getCommand(nextCommand, reef, reefHeight, source, sourceDistance));
      }
      first = !first;
    }
    SequentialCommandGroup commands = new SequentialCommandGroup();
    for (int i = 0; i < commandList.size(); i++) {
      commands.addCommands(commandList.get(i));
    }
    return commands;
  }

  private Command getCommand(
      NextCommand nextCommand,
      Reef reef,
      ReefHeight reefHeight,
      Source source,
      SourceDistance sourceDistance) {

    if (nextCommand == null) {
      return new Command() {};
    }
    switch (nextCommand) {
      case PLACE:
        return createPlaceCommand(reef, reefHeight);
      case SOURCE:
        return createSourceCommand(source, sourceDistance);
      default:
        return new Command() {};
    }
  }

  private Command createPlaceCommand(Reef reef, ReefHeight reefHeight) {
    return Commands.runOnce(() -> superstructure.setTargets(reef, reefHeight))
        .andThen(
            Commands.runOnce(() -> superstructure.setWantedState(WantedState.AUTO_DRIVE_TO_REEF)))
        .andThen(new WaitUntilCommand(superstructure::isPathFindingFinishedAuto))
        .andThen(Commands.runOnce(() -> superstructure.setWantedState(WantedState.SCORE_AUTO)))
        .andThen(
            new WaitUntilCommand(
                () -> superstructure.getWantedState() == WantedState.DEFAULT_STATE));
  }

  private Command createSourceCommand(Source source, SourceDistance sourceDistance) {
    return Commands.runOnce(() -> superstructure.setTargets(source, sourceDistance))
        .andThen(
            Commands.runOnce(
                () -> superstructure.setWantedState(WantedState.AUTO_DRIVE_TO_CORAL_STATION)))
        .andThen(new WaitUntilCommand(superstructure::isPathFindingFinishedAuto))
        .andThen(
            Commands.runOnce(
                () -> superstructure.setWantedState(WantedState.INTAKE_CORAL_FROM_STATION)))
        .andThen(
            new WaitUntilCommand(
                () -> superstructure.getWantedState() == WantedState.DEFAULT_STATE));
  }

  public enum Auto {
    IDLE,
    E4RFD4RFC4RMB4,
    J4NFK4NFL4NMA4,
    CUSTOM,
  }

  public enum NextCommand {
    PLACE,
    SOURCE,
  }

  public enum Source {
    RIGHT,
    LEFT,
  }

  public enum SourceDistance {
    FAR,
    MID,
    CLOSE,
  }

  public enum Reef {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
  }

  public enum ReefHeight {
    L4,
    L3,
    L2,
  }
}
