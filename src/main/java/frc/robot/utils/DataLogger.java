// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * DataLogger
 * 
 * Designed to be an easy interface to log various bits of data like
 * power usage, motor output, temperatures, sensor values, and mechanism states in 
 * an onboard log that can be analysed after a match or test session
 */
public class DataLogger {
  private static DataLogger m_logger = null;
  private static DataLog m_log;
  private static List<LogEntry> m_logEntries;
  public static final Command LOGGING_COMMAND = new RunCommand(() -> m_logger.log(), new Subsystem() {});

  public static class LogEntry {
    private DataLogEntry entry;
    private Runnable logValue;
    
    /**
     * Log Entry for integers
     * @param supplier source of integer value
     * @param name identifier string
     */
    public LogEntry(IntSupplier supplier, String name) {
      entry = new IntegerLogEntry(m_log, name);
      logValue = () -> {
        ((IntegerLogEntry)entry).append(supplier.getAsInt());
      };
    }

    /**
     * Log Entry for doubles
     * @param supplier source of double value
     * @param name identifier string
     */
    public LogEntry(DoubleSupplier supplier, String name) {
      entry = new DoubleLogEntry(m_log, name);
      logValue = () -> {
        ((DoubleLogEntry)entry).append(supplier.getAsDouble());
      };
    }

    /**
     * Log Entry for booleans
     * @param supplier source of boolean value
     * @param name identifier string
     */
    public LogEntry(BooleanSupplier supplier, String name) {
      entry = new BooleanLogEntry(m_log, name);
      logValue = () -> {
        ((BooleanLogEntry)entry).append(supplier.getAsBoolean());
      };
    }

    /**
     * Log Entry for strings
     * @param supplier source of string value
     * @param name identifier string
     */
    public LogEntry(Supplier<String> supplier, String name) {
      entry = new StringLogEntry(m_log, name);
      logValue = () -> {
        ((StringLogEntry)entry).append(supplier.get());
      };
    }

    public void log() {
      logValue.run();
    }
  }

  private DataLogger() {
    m_log = DataLogManager.getLog();
  }

  /**
   * Get instance of DataLogger, creating if nonexistent
   * @return global DataLogger instance
   */
  public static DataLogger getInstance() {
    if (m_logger == null) m_logger = new DataLogger();
    return m_logger;
  }

  /**
   * Start logging
   */
  public void startLogging() {
    m_logger.startLogging();
  }

  /**
   * Add entry to be logged
   * @param entry desired entry
   */
  public void addEntry(LogEntry entry) {
    m_logEntries.add(entry);
  }

  /**
   * Log all values with current timestamp
   */
  public void log() {
    for (LogEntry entry : m_logEntries) entry.log();
  }
}
