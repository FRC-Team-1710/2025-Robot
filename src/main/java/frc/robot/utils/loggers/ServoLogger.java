package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Servo;

@CustomLoggerFor(Servo.class)
public class ServoLogger extends ClassSpecificLogger<Servo> {
  public ServoLogger() {
    super(Servo.class);
  }

  @Override
  public void update(EpilogueBackend backend, Servo servo) {
    backend.log("Angle", servo.getAngle());
    backend.log("Channel", servo.getChannel());
    backend.log("Speed", servo.getSpeed());
  }
}
