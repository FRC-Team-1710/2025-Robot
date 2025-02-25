package frc.robot.utils;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged;
import java.util.Arrays;

public class ArrayBuilder {
  /**
   * Builds an array of `SwerveModuleState` objects.
   *
   * @param size The number of elements in the array.
   * @return An initialized array of `SwerveModuleState` objects.
   */
  public static SwerveModuleState[] buildSwerveModuleState() {
    if (Constants.PP_CONFIG.numModules <= 0) {
      throw new IllegalArgumentException("Size must be positive");
    }
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.PP_CONFIG.numModules];
    Arrays.setAll(moduleStates, i -> new SwerveModuleState());
    return moduleStates;
  }

  /**
   * Builds an array of `SwerveModulePosition` objects based on number of modules.
   *
   * @return An initialized array of `SwerveModulePosition` objects.
   */
  public static SwerveModulePosition[] buildSwerveModulePosition() {
    if (Constants.PP_CONFIG.numModules <= 0) {
      throw new IllegalArgumentException("Size must be positive");
    }
    SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[Constants.PP_CONFIG.numModules];
    Arrays.setAll(modulePositions, i -> new SwerveModulePosition());
    return modulePositions;
  }

  /**
   * Builds an array of `ModuleIOInputsAutoLogged` objects based on number of modules.
   *
   * @return An initialized array of `ModuleIOInputsAutoLogged` objects.
   */
  public static ModuleIOInputsAutoLogged[] buildModuleAutoLogged() {
    if (Constants.PP_CONFIG.numModules <= 0) {
      throw new IllegalArgumentException("Size must be positive");
    }
    ModuleIOInputsAutoLogged[] moduleAutoLogged =
        new ModuleIOInputsAutoLogged[Constants.PP_CONFIG.numModules];
    Arrays.setAll(moduleAutoLogged, i -> new ModuleIOInputsAutoLogged());
    return moduleAutoLogged;
  }

  /**
   * Builds an array of `Alert` objects based on number of modules.
   *
   * @param alarmText The text to display on alarm.
   * @return An initialized array of `Alert` objects.
   */
  public static Alert[] buildAlert(String alarmText) {
    if (Constants.PP_CONFIG.numModules <= 0) {
      throw new IllegalArgumentException("Size must be positive");
    }
    Alert[] alerts = new Alert[Constants.PP_CONFIG.numModules];
    Arrays.setAll(
        alerts, i -> new Alert(alarmText + " " + Integer.toString(i) + ".", AlertType.kError));
    return alerts;
  }
}
