package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Timer;

@CustomLoggerFor(Timer.class)
public class TimerLogger extends ClassSpecificLogger<Timer> {
  public TimerLogger() {
    super(Timer.class);
  }

  @Override
  public void update(EpilogueBackend backend, Timer timer) {
    backend.log("Time(sec)", timer.get() / 1000);
  }
}
