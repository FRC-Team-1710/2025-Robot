package frc.robot.utils.customLogger.backend;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Measure;
import java.util.HashMap;
import java.util.Map;

public class NTSimplyLoggerBackend implements SimplyLoggerBackend {
  private final NetworkTableInstance m_nt;

  private final Map<String, Publisher> m_publishers = new HashMap<>();

  public NTSimplyLoggerBackend(NetworkTableInstance nt) {
    this.m_nt = nt;
  }

  @Override
  @SuppressWarnings("rawtypes")
  public void log(String identifier, Object value) {
    if (value instanceof Enum) {
      log(identifier, (Enum) value);
    }
  }

  @Override
  public void log(String identifier, int value) {
    ((IntegerPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getIntegerTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, long value) {
    ((IntegerPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getIntegerTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, float value) {
    ((FloatPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getFloatTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, double value) {
    ((DoublePublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getDoubleTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, boolean value) {
    ((BooleanPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getBooleanTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, byte[] value) {
    ((RawPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getRawTopic(k).publish("raw")))
        .set(value);
  }

  @Override
  @SuppressWarnings("PMD.UnnecessaryCastRule")
  public void log(String identifier, int[] value) {
    long[] widened = new long[value.length];

    for (int i = 0; i < value.length; i++) {
      widened[i] = (long) value[i];
    }

    ((IntegerArrayPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getIntegerArrayTopic(k).publish()))
        .set(widened);
  }

  @Override
  public void log(String identifier, long[] value) {
    ((IntegerArrayPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getIntegerArrayTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, float[] value) {
    ((FloatArrayPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getFloatArrayTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, double[] value) {
    ((DoubleArrayPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getDoubleArrayTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, boolean[] value) {
    ((BooleanArrayPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getBooleanArrayTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, String value) {
    ((StringPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getStringTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, String[] value) {
    ((StringArrayPublisher)
            m_publishers.computeIfAbsent(identifier, k -> m_nt.getStringArrayTopic(k).publish()))
        .set(value);
  }

  @Override
  public void log(String identifier, Measure<?> value) {
    // ((su)
    //         m_publishers.computeIfAbsent(identifier, k -> m_nt.getStringArrayTopic(k).publish()))
    //     .set(value);
  }
}
