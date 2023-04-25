// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase implements AutoCloseable {

  private static LEDSubsystem m_subsystem;

  private Set<LEDStrip> m_ledStrips;

  /** Creates a new LEDSubsystem. */
  private LEDSubsystem() {
    m_ledStrips = new HashSet<LEDStrip>();
  }

  public static LEDSubsystem getInstance() {
    if (m_subsystem == null) m_subsystem = new LEDSubsystem();
    return m_subsystem;
  }

  private void setLEDs() {
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.update();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setLEDs();
  }

  public void add(LEDStrip... ledStrips) {
    for (LEDStrip ledStrip : ledStrips) m_ledStrips.add(ledStrip);
  }

  @Override
  public void close() {
    for (LEDStrip ledStrip : m_ledStrips) ledStrip.close();
  }
}
