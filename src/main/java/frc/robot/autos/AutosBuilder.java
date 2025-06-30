// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

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

  public AutosBuilder(Superstructure superstructure) {
    this.superstructure = superstructure;
    SmartDashboard.putString("Custom Auto Input", "(insert auto here)");
    SmartDashboard.putString("Custom Auto Input Key",
        "(A-L=Pipe,2-4=Level),(RN=RightOrLeftSource,FMC=FarOrMidOrCloes)");
    autoChooser.addDefaultOption("IDLE", Auto.IDLE);
    autoChooser.addOption("CUSTOM", Auto.CUSTOM);
  }

  public Command getAuto() {
    switch (autoChooser.get()) {
      case CUSTOM:
        if (SmartDashboard.getString("Custom Auto Input", "(insert auto here)") == "(insert auto here)") {
          return Commands.runOnce(() -> superstructure.setWantedState(WantedState.ZERO));
        } else {
          return buildCustomAuto(SmartDashboard.getString("Custom Auto Input", "(insert auto here)"));
        }
      default:
        return Commands.runOnce(() -> superstructure.setWantedState(WantedState.ZERO));
    }
  }

  private Command buildCustomAuto(String input) {
    boolean first = true;
    int more;
    for (int i = 0; i < input.length(); i++) {
      String character = String.valueOf(input.charAt(i));
      if (first) {
        if (character == "R") {
          nextCommand = NextCommand.SOURCE;
          source = Source.RIGHT;
        } else if (character == "N") {
          nextCommand = NextCommand.SOURCE;
          source = Source.LEFT;
        } else if (character == "A") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.A;
        } else if (character == "B") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.B;
        }else if (character == "C") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.C;
        } else if (character == "D") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.D;
        }else if (character == "E") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.E;
        } else if (character == "F") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.F;
        }else if (character == "G") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.G;
        } else if (character == "H") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.H;
        }else if (character == "I") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.I;
        } else if (character == "J") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.J;
        }else if (character == "K") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.K;
        } else if (character == "L") {
          nextCommand = NextCommand.PLACE;
          reef = Reef.L;
        } else {
          System.out.println("First half command wasn't real");
        }
      } else {
        if (character == "2") {
          reefHeight = ReefHeight.L2;
        } else if (character == "3") {
          reefHeight = ReefHeight.L3;
        } else if (character == "4") {
          reefHeight = ReefHeight.L4;
        } else if (character == "F") {
          sourceDistance = SourceDistance.FAR;
        } else if (character == "M") {
          sourceDistance = SourceDistance.MID;
        } else if (character == "C") {
          sourceDistance = SourceDistance.CLOSE;
        }
        commandList.add(getCommand());
      }
      first = !first;
    }
  }

  private Command getCommand() {
    switch (nextCommand) {
      case PLACE:
        return createPlaceCommand();
      case SOURCE:
        return createSourceCommand();
    }
  }

  private Command createPlaceCommand() {
    return Commands.runOnce(() -> superstructure.set);
  }

  public enum Auto {
    IDLE, CUSTOM,
  }

  public enum NextCommand {
    PLACE, SOURCE,
  }

  public enum Source {
    RIGHT, LEFT,
  }

  public enum SourceDistance {
    FAR, MID, CLOSE,
  }

  public enum Reef {
    A, B, C, D, E, F, G, H, I, J, K, L,
  }

  public enum ReefHeight {
    L4,
    L3,
    L2,
  }
}
