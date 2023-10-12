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
  private static final String LAST_BATTERY_PATH = "last-battery.txt";

  public static Boolean isBatteryReused() { // Checks if the battery is reused based on the last battery on file, run only if hardware real?
    File file = new File(LAST_BATTERY_PATH);

    if (file.exists()) {
      // Read previous battery name
      String previousBatteryName = "";
      try {
          previousBatteryName =
            new String(Files.readAllBytes(Paths.get(LAST_BATTERY_PATH)), StandardCharsets.UTF_8);
      } catch (IOException e) {
        // Throw error
        e.printStackTrace();
      }

      if (previousBatteryName.equals(BatteryScanner.scanBattery())) {
        return true;
      } else {
        // New battery, delete file
        file.delete();
      }
    }

    return false;
  }

  public static void writeCurrentBattery() { // Write current battery, only if hardware real and battery attached?
    try {
      FileWriter fileWriter = new FileWriter(LAST_BATTERY_PATH);
      fileWriter.write(BatteryScanner.scanBattery());
      fileWriter.close();
    } catch (IOException e) {
      // Throw error
      e.printStackTrace();
    }
  }
}
