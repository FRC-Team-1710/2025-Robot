package frc.robot.utils.customLogger.backend;

import edu.wpi.first.units.Measure;
import java.util.Collection;

public interface SimplyLoggerBackend {
  void log(String identifier, Object value);

  void log(String identifier, int value);

  void log(String identifier, long value);

  void log(String identifier, float value);

  void log(String identifier, double value);

  void log(String identifier, boolean value);

  void log(String identifier, byte[] value);

  void log(String identifier, int[] value);

  void log(String identifier, long[] value);

  void log(String identifier, float[] value);

  void log(String identifier, double[] value);

  void log(String identifier, boolean[] value);

  void log(String identifier, String value);

  void log(String identifier, String[] value);

  void log(String identifier, Measure<?> value);

  default void log(String identifier, Collection<String> value) {
    log(identifier, value.toArray(String[]::new));
  }

  default void log(String identifier, Enum<?> value) {
    log(identifier, value.name());
  }
}
