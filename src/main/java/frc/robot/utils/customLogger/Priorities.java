package frc.robot.utils.customLogger;

public enum Priorities {
  DEBUG(3),
  TESTING(2),
  COMP(1),
  OVERRIDE(0);

  private final int logPriority;

  Priorities(int logPriority) {
    this.logPriority = logPriority;
  }

  public boolean shouldLog(Priorities priority) {
    return this.logPriority <= priority.logPriority;
  }
}
