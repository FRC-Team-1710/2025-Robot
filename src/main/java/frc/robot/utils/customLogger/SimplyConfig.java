package frc.robot.utils.customLogger;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import frc.robot.utils.customLogger.backend.NTSimplyLoggerBackend;
import frc.robot.utils.customLogger.backend.SimplyLoggerBackend;

public class SimplyConfig {
  public SimplyLoggerBackend backend = new NTSimplyLoggerBackend(NetworkTableInstance.getDefault());

  public Time loggingPeriod = Milliseconds.of(20);

  public Time loggingPeriodOffset = Milliseconds.of(0);

  public Priorities minimumProperty = Priorities.DEBUG;

  public String root = "Robot";

  public SimplyConfig() {}

  public SimplyConfig withBackend(SimplyLoggerBackend backend) {
    this.backend = backend;
    return this;
  }

  public SimplyConfig withLoggingPeriod(Time loggingPeriod) {
    this.loggingPeriod = loggingPeriod;
    return this;
  }

  public SimplyConfig withLoggingPeriodOffset(Time loggingPeriodOffset) {
    this.loggingPeriodOffset = loggingPeriodOffset;
    return this;
  }

  public SimplyConfig withMinimumProperty(Priorities minimumProperty) {
    this.minimumProperty = minimumProperty;
    return this;
  }

  public SimplyConfig withRoot(String root) {
    this.root = root;
    return this;
  }
}
