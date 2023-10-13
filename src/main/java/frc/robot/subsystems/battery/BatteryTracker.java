// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.battery;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

public class BatteryTracker {
  private static final String PREVIOUS_BATTERY_PATH = "previous_battery.txt";

  public static Boolean isBatteryReused() { // Checks if the battery is reused based on the previous battery on file, run only if hardware real?
    File file = new File(PREVIOUS_BATTERY_PATH);

    if (!file.exists()) return false;
    
    // Read previous battery name
    String previousBatteryID = "";
    try {
        previousBatteryID =
          new String(Files.readAllBytes(Paths.get(PREVIOUS_BATTERY_PATH)), StandardCharsets.UTF_8);
    } catch (IOException e) {
      // Throw error
      e.printStackTrace();
    }

    if (previousBatteryID.equals(BatteryScanner.scanBattery())) {
      return true;
    } else {
      // New battery, delete file
      file.delete();
      return false;
    }
  }

  /**
   * Write current battery
   */
  public static void writeCurrentBattery() {
    try {
      FileWriter fileWriter = new FileWriter(PREVIOUS_BATTERY_PATH);
      fileWriter.write(BatteryScanner.scanBattery());
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
