package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.Command;

@CustomLoggerFor(Command.class)
public class CommandLogger extends ClassSpecificLogger<Command> {
  public CommandLogger() {
    super(Command.class);
  }

  @Override
  public void update(EpilogueBackend backend, Command command) {
    backend.log("Name", command.getName());
    backend.log("Subsystem", command.getSubsystem());
    backend.log("IsFinished", command.isFinished());
    backend.log("IsScheduled", command.isScheduled());
    backend.log("ToString", command.toString());
  }
}
