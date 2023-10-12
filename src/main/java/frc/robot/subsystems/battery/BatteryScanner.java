// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.battery;

import edu.wpi.first.wpilibj.SerialPort;

public class BatteryScanner {
  // Battery name
  private static final int BATTERY_NAME_LENGTH = 8;

  // Scanner
  private static final byte[] SCAN_COMMNAND = 
    new byte[] {0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, (byte) 0xab, (byte) 0xcd};
  private static final byte[] RESPONSE_PREFIX = 
    new byte[] {0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31};
  private static final int RESPONSE_LENGTH = RESPONSE_PREFIX.length + BATTERY_NAME_LENGTH;
  private static final int BAUD_RATE = 9600;
  private static final double TIMEOUT = 1.5;

  /**
   * Scan current battery
   */
  public static String scanBattery() {
    String m_batteryName = "";

    try (SerialPort port = new SerialPort(BAUD_RATE, SerialPort.Port.kUSB)) {
      port.setTimeout(TIMEOUT);
      port.setWriteBufferSize(SCAN_COMMNAND.length);
      port.setReadBufferSize(RESPONSE_LENGTH);
      port.write(SCAN_COMMNAND, SCAN_COMMNAND.length);
      byte[] response = port.read(RESPONSE_LENGTH);

      // Ensure response is correct length
      if (response.length != RESPONSE_LENGTH) {
        // Throw Error
        return m_batteryName;
      }

      // Ensure response starts with prefix
      for (int i = 0; i < RESPONSE_PREFIX.length; i++) {
        if (response[i] != RESPONSE_PREFIX[i]) {
          // Throw Error
          return m_batteryName;
        }
      }

      // Read name from data
      byte[] batteryNameBytes = new byte[BATTERY_NAME_LENGTH];
      System.arraycopy(response, RESPONSE_PREFIX.length, batteryNameBytes, 0, BATTERY_NAME_LENGTH);
      m_batteryName = new String(batteryNameBytes);
    } catch (Exception e) {
      // Throw Error
      e.printStackTrace();
    }

    return m_batteryName;
  }
}