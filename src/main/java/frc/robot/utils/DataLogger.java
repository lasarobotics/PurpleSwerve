package frc.robot.utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;

public class DataLogger {
  private static final String TABLE_NAME = "data";
  private static final String METADATA = "metadata";

  public static class StringLogEntry extends edu.wpi.first.util.datalog.StringLogEntry {
    StringPublisher m_valuePublisher;
    StringPublisher m_metadataPublisher;
    public StringLogEntry(DataLog log, String name, String metadata) {
      super(log, name, metadata, 0);
      m_valuePublisher = NetworkTableInstance.getDefault()
        .getTable(TABLE_NAME).getStringTopic(name).publish();
      m_metadataPublisher = NetworkTableInstance.getDefault()
        .getTable(TABLE_NAME).getStringTopic(String.join(" ", name, METADATA)).publish();
    }

    public StringLogEntry(DataLog log, String name) {
      this(log, name, "");
    }

    public void append(String value) {
      append(value, "");
    }

    public void append(String value, String metadata) {
      super.setMetadata(metadata);
      super.append(value);
      m_metadataPublisher.set(metadata);
      m_valuePublisher.set(value);
    }
  }

  public static class BooleanLogEntry extends edu.wpi.first.util.datalog.BooleanLogEntry {
    BooleanPublisher m_valuePublisher;
    StringPublisher m_metadataPublisher;
    public BooleanLogEntry(DataLog log, String name, String metadata) {
      super(log, name, metadata, 0);
      m_valuePublisher = NetworkTableInstance.getDefault()
        .getTable(TABLE_NAME).getBooleanTopic(name).publish();
      m_metadataPublisher = NetworkTableInstance.getDefault()
        .getTable(TABLE_NAME).getStringTopic(String.join(" ", name, METADATA)).publish();
    }

    public BooleanLogEntry(DataLog log, String name) {
      this(log, name, "");
    }

    public void append(boolean value) {
      append(value, "");
    }

    public void append(boolean value, String metadata) {
      super.setMetadata(metadata);
      super.append(value);
      m_metadataPublisher.set(metadata);
      m_valuePublisher.set(value);
    }
  }

  public static class DoubleLogEntry extends edu.wpi.first.util.datalog.DoubleLogEntry {
    DoublePublisher m_valuePublisher;
    StringPublisher m_metadataPublisher;
    public DoubleLogEntry(DataLog log, String name, String metadata) {
      super(log, name, metadata, 0);
      m_valuePublisher = NetworkTableInstance.getDefault()
        .getTable(TABLE_NAME).getDoubleTopic(name).publish();
      m_metadataPublisher = NetworkTableInstance.getDefault()
        .getTable(TABLE_NAME).getStringTopic(String.join(" ", name, METADATA)).publish();
    }

    public DoubleLogEntry(DataLog log, String name) {
      this(log, name, "");
    }

    public void append(double value) {
      append(value, "");
    }

    public void append(double value, String metadata) {
      super.setMetadata(metadata);
      super.append(value);
      m_metadataPublisher.set(metadata);
      m_valuePublisher.set(value);
    }
  }

  public static void log(String message) {
    DataLogManager.log(message);
  }
}