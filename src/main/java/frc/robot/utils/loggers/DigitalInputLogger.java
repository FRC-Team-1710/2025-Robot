package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@CustomLoggerFor(DigitalInput.class)
  public class DigitalInputLogger extends ClassSpecificLogger<DigitalInput> {
    public DigitalInputLogger() {
      super(DigitalInput.class);
    }
  
    @Override
    public void update(EpilogueBackend backend, DigitalInput input) {
      backend.log("Broken", input.get());
      backend.log("Channel", input.getChannel());
    }
  }