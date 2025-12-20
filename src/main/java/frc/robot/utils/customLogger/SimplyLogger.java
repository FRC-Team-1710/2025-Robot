package frc.robot.utils.customLogger;

import java.lang.reflect.Field;

import com.fasterxml.jackson.databind.introspect.AnnotatedClass;

import edu.wpi.first.wpilibj.DriverStation;

public class SimplyLogger {
  private final SimplyConfig config;

  public SimplyLogger() {
    this.config = new SimplyConfig();
  }

  public SimplyLogger(SimplyConfig config) {
    this.config = config;
  }

  public void periodic() {
    
  }
}
