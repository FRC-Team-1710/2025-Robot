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

  private String autoString = "cooked";

  private Command preBuiltAuto = Commands.none();

  HashMap<Character, Source> charToSource = new HashMap<Character, Source>();
  HashMap<Character, Reef> charToReef = new HashMap<Character, Reef>();
  HashMap<Character, ReefHeight> charToReefHeight = new HashMap<Character, ReefHeight>();
  HashMap<Character, SourceDistance> charToSourceDistance =
      new HashMap<Character, SourceDistance>();

  public AutosBuilder(Superstructure superstructure) {
    // Add keys & values to HashMap
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

    // Add defaults to SmartDashboard

    SmartDashboard.putString("Custom Auto Input", "(insert auto here)");
    SmartDashboard.putString(
        "Custom Auto Input Key", "(A-L=Pipe,2-4=Level),(RN=RightOrLeftSource,FMC=FarOrMidOrCloes)");

    autoChooser.addDefaultOption("IDLE", Auto.IDLE);
    autoChooser.addOption("CUSTOM", Auto.CUSTOM);
    for (Auto auto : Auto.values()) {
      if (auto != Auto.IDLE && auto != Auto.CUSTOM) {
        autoChooser.addOption(auto.toString(), auto);
      }
    }
  }

  public void periodic() {
    // If it's set to custom and the custom is diffrent, build the auto
    if (autoChooser.get() == Auto.CUSTOM
        && autoString != SmartDashboard.getString("Custom Auto Input", "(insert auto here)")) {
      autoString = SmartDashboard.getString("Custom Auto Input", "(insert auto here)");
      String output = validateAuto(autoString);
      Logger.recordOutput("Is Auto Valid", output == "");
      Logger.recordOutput("Auto Validation Error", output);
      preBuiltAuto = output == "" ? buildAuto() : Commands.none();
    } else if (autoChooser.get() != Auto.CUSTOM
        && autoChooser.get() != Auto.IDLE
        && autoString != autoChooser.get().toString()) {
      autoString = autoChooser.get().toString();
      String output = validateAuto(autoString);
      Logger.recordOutput("Is Auto Valid", output == "");
      Logger.recordOutput("Auto Validation Error", output);
      preBuiltAuto = output == "" ? buildAuto() : Commands.none();
    } else if (autoChooser.get() == Auto.IDLE) {
      Logger.recordOutput("Is Auto Valid", true);
      Logger.recordOutput("Auto Validation Error", "bum");
      preBuiltAuto = buildAuto();
    }
  }

  public String validateAuto(String input) {
    boolean first = true;
    for (int i = 0; i < input.length(); i++) {
      char character = input.charAt(i);
      // Sets the NextCommand and source to specific value from HashMap
      // For first character in pair
      if (first) {
        if (charToSource.containsKey(character)) {
          nextCommand = NextCommand.SOURCE;
          source = charToSource.get(character);
        } else if (charToReef.containsKey(character)) {
          nextCommand = NextCommand.PLACE;
          reef = charToReef.get(character);
        } else {
          // Return the error
          return "Character at character "
              + (i + 1)
              + " of the first half was "
              + String.valueOf(character)
              + " which is invalid";
        }
      } else {
        // Also sets NextCommand (see comment above)
        // For second character in pair
        if (nextCommand == NextCommand.PLACE) {
          if (charToReefHeight.containsKey(character)) {
            reefHeight = charToReefHeight.get(character);
          } else {
            // Return the error
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
            // Returns the error
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
      first = !first; // Swaps whether it's first or second character in pair
    }
    return ""; // Returns nothing if no errors found
  }

  /**
   * @return cached auto that was built in periodic
   */
  public Command getAuto() {
    return autoChooser.get() == Auto.CUSTOM
        ? preBuiltAuto
        : buildAuto(); // Return preBuiltAuto if custom, else build auto normally
  }

  /**
   * Builds auto based on auto chooser
   *
   * @return command to schedule for auto
   */
  public Command buildAuto() {
    commandList = new ArrayList<>();
    if (Constants.currentMode == Mode.SIM) {
      commandList.add(Commands.runOnce(() -> superstructure.beginSimAuto()));
    }
    switch (autoChooser.get()) {
      case CUSTOM:
        if (SmartDashboard.getString("Custom Auto Input", "(insert auto here)")
            == "(insert auto here)") {
          return Commands.runOnce(
              () ->
                  superstructure.setWantedState(
                      WantedState.DEFAULT_STATE)); // Returns zero command if no custom input
        } else {
          return buildAuto(
              SmartDashboard.getString(
                  "Custom Auto Input",
                  "(insert auto here)")); // returns built custom auto from SmartDashboard input
        }
      case IDLE:
        return Commands.runOnce(
            () ->
                superstructure.setWantedState(
                    WantedState.DEFAULT_STATE)); // Sets wanted state to zero when idle
      default:
        return buildAuto(autoString);
    }
  }

  /**
   * Builds auto from the input
   *
   * @param input string to build auto from
   * @return command to schedule for auto
   */
  private Command buildAuto(String input) {
    // Same as validateAuto but builds the command list instead of returning errors
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

    // Adds all the commands in commandList to a SequentialCommandGroup and returns it
    SequentialCommandGroup commands = new SequentialCommandGroup();
    for (int i = 0; i < commandList.size(); i++) {
      commands.addCommands(commandList.get(i));
    }
    return commands;
  }

  // Turns the enums into actual commands
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

  // Command for placing coral on reef, takes reef side and reefheight as parameters
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

  // Command for sourcing coral from station, takes near/far source and distance as parameters
  private Command createSourceCommand(Source source, SourceDistance sourceDistance) {
    return Commands.runOnce(() -> superstructure.setTargets(source, sourceDistance))
        .andThen(
            Commands.runOnce(
                () -> superstructure.setWantedState(WantedState.INTAKE_CORAL_FROM_STATION)))
        .andThen(
            new WaitUntilCommand(
                () -> superstructure.getWantedState() == WantedState.DEFAULT_STATE));
  }

  // Enums
  public enum Auto {
    IDLE,
    E4RFD4RFC4RMB4,
    J4NFK4NFL4NMA4,
    K4NFL4NMA4NM,
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
