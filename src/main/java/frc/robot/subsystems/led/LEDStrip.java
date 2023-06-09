// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/** LED Strip */
public class LEDStrip implements AutoCloseable {
  public static final Color TEAM_COLOR = new Color(0x66, 0x33, 0x99);

  public static class Hardware {
    boolean isHardwareReal;
    AddressableLED ledStrip;

    public Hardware(boolean isHardwareReal, AddressableLED ledStrip) {
      this.isHardwareReal = isHardwareReal;
      this.ledStrip = ledStrip;
    }
  }

  /**
   * LED strip sections
   */
  public static enum Section {
    START,
    MIDDLE,
    END;

    public static final Section FULL[] = { START, MIDDLE, END };

    private static final int SMALL_SECTION_LENGTH = 10;

    /**
     * Get start index of LED strip sections
     * @param buffer LED buffer
     * @param sections Desired sections
     * @return Start index
     */
    private static int start(AddressableLEDBuffer buffer, Section... sections) {
      return start(
        buffer, 
        Collections.min(
          Arrays.asList(sections), 
          (a, b) -> Integer.compare(a.ordinal(), b.ordinal())
        )
      );
    }
    
    /**
     * Get start index of LED strip section
     * @param buffer LED buffer
     * @param section Desired section
     * @return Start index
     */
    private static int start(AddressableLEDBuffer buffer, Section section) {
      switch (section) {
        case START:
          return 0;
        case MIDDLE:
          return SMALL_SECTION_LENGTH;
        case END:
          return buffer.getLength() - SMALL_SECTION_LENGTH;
        default:
          return 0;
      }
    }
    
    /**
     * Get end index of LED strip sections
     * @param buffer LED buffer
     * @param sections Desired sections
     * @return End index
     */
    private static int end(AddressableLEDBuffer buffer, Section... sections) {
      return end(
        buffer, 
        Collections.max(
          Arrays.asList(sections), 
          (a, b) -> Integer.compare(a.ordinal(), b.ordinal())
        )
      );
    }
    
    /**
     * Get end index of LED strip section
     * @param buffer LED buffer
     * @param sections Desired section
     * @return end index
     */
    private static int end(AddressableLEDBuffer buffer, Section section) {
      switch (section) {
        case START:
          return SMALL_SECTION_LENGTH;
        case MIDDLE:
          return buffer.getLength() - SMALL_SECTION_LENGTH;
        case END:
          return buffer.getLength();
        default:
          return buffer.getLength();
      }
    }

    /**
     * Check if index is within LED strip sections
     * @param i Index
     * @param buffer LED buffer
     * @param sections Desired sections
     * @return True if index falls within specified sections
     */
    private static boolean contains(int i, AddressableLEDBuffer buffer, Section... sections) {
      boolean contains = false;
      for (Section section : sections) {
        contains |= Section.start(buffer, section) <= i && i < Section.end(buffer, section);
        if (contains) break;
      }

      return contains;
    } 
  }

  /**
   * Valid LED patterns
   */
  public enum Pattern {
    // Team color patterns
    TEAM_COLOR_SOLID(PatternType.SOLID, TEAM_COLOR),
    TEAM_COLOR_STROBE(PatternType.STROBE, TEAM_COLOR),
    TEAM_COLOR_BREATHE(PatternType.BREATHE, TEAM_COLOR),
    TEAM_COLOR_WAVE(PatternType.WAVE, TEAM_COLOR),
    // Red patterns
    RED_SOLID(PatternType.SOLID, Color.kRed),
    RED_STROBE(PatternType.STROBE, Color.kRed),
    RED_BREATHE(PatternType.BREATHE, Color.kRed),
    RED_WAVE(PatternType.WAVE, Color.kRed),
    // Orange patterns
    ORANGE_SOLID(PatternType.SOLID, Color.kOrange),
    ORANGE_STROBE(PatternType.STROBE, Color.kOrange),
    ORANGE_BREATHE(PatternType.BREATHE, Color.kOrange),
    ORANGE_WAVE(PatternType.WAVE, Color.kOrange),
    // Yellow patterns
    YELLOW_SOLID(PatternType.SOLID, Color.kYellow),
    YELLOW_STROBE(PatternType.STROBE, Color.kYellow),
    YELLOW_BREATHE(PatternType.BREATHE, Color.kYellow),
    YELLOW_WAVE(PatternType.WAVE, Color.kYellow),
    // Green patterns
    GREEN_SOLID(PatternType.SOLID, Color.kGreen),
    GREEN_STROBE(PatternType.STROBE, Color.kGreen),
    GREEN_BREATHE(PatternType.BREATHE, Color.kGreen),
    GREEN_WAVE(PatternType.WAVE, Color.kGreen),
    // Blue patterns
    BLUE_SOLID(PatternType.SOLID, Color.kBlue),
    BLUE_STROBE(PatternType.STROBE, Color.kBlue),
    BLUE_BREATHE(PatternType.BREATHE, Color.kBlue),
    BLUE_WAVE(PatternType.WAVE, Color.kBlue),
    // Indigo patterns
    INDIGO_SOLID(PatternType.SOLID, Color.kIndigo),
    INDIGO_STROBE(PatternType.STROBE, Color.kIndigo),
    INDIGO_BREATHE(PatternType.BREATHE, Color.kIndigo),
    INDIGO_WAVE(PatternType.WAVE, Color.kIndigo),
    // Violet patterns
    VIOLET_SOLID(PatternType.SOLID, Color.kViolet),
    VIOLET_STROBE(PatternType.STROBE, Color.kViolet),
    VIOLET_BREATHE(PatternType.BREATHE, Color.kViolet),
    VIOLET_WAVE(PatternType.WAVE, Color.kViolet),
    // Pink patterns
    PINK_SOLID(PatternType.SOLID, Color.kPink),
    PINK_STROBE(PatternType.STROBE, Color.kPink),
    PINK_BREATHE(PatternType.BREATHE, Color.kPink),
    PINK_WAVE(PatternType.WAVE, Color.kPink),
    // Special patterns
    OFF(PatternType.SOLID, Color.kBlack),
    RAINBOW(PatternType.RAINBOW, Color.kBlack);

    public final PatternType type;
    public final Color color;

    private Pattern(PatternType type, Color color) {
      this.type = type;
      this.color = color;
    }
  }

  private enum PatternType {
    SOLID, STROBE, BREATHE, WAVE, RAINBOW;
  }

  private AddressableLED m_leds;
  private AddressableLEDBuffer m_buffer;
  private HashMap<Section[], Pattern> m_sectionLEDPatterns, m_tempLEDPatterns;

  private static final double STROBE_DURATION = 0.1;
  private static final double BREATHE_DURATION = 1.0;
  private static final double RAINBOW_CYCLE_LENGTH = 25.0;
  private static final double RAINBOW_DURATION = 0.25;
  private static final double WAVE_EXPONENT = 0.4;
  private static final double WAVE_CYCLE_LENGTH = 25.0;
  private static final double WAVE_DURATION = 1.0;

  /**
   * Create an instance of an LED strip
   * @param ledStripHardware Hardware devices required by LED strip
   * @param length Number of LEDs in strip
   */
  public LEDStrip(Hardware ledStripHardware, int length) {
    this.m_leds = ledStripHardware.ledStrip;
    this.m_buffer = new AddressableLEDBuffer(length);
    this.m_sectionLEDPatterns = new HashMap<Section[], Pattern>();
    this.m_tempLEDPatterns = new HashMap<Section[], Pattern>();

    m_sectionLEDPatterns.put(Section.FULL, Pattern.TEAM_COLOR_SOLID);
  }

  /**
   * Initialize hardware devices for LED strip
   * 
   * @param ledStripPort PWM port for LED strip
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware(boolean isHardwareReal, int ledStripPort) {
    Hardware ledStripHardware = new Hardware(isHardwareReal, new AddressableLED(ledStripPort));

    return ledStripHardware;
  }

  /**
   * Set LED strip section to solid color
   * @param sections Section of LED strip
   * @param color Color to set
   */
  private void solid(Color color, Section... sections) {
    for (int i = Section.start(m_buffer, sections); i < Section.end(m_buffer, sections); i++) {
      if (Section.contains(i, m_buffer, sections)) m_buffer.setLED(i, color);
    }
  }

  /**
   * Set LED strip section to strobe pattern
   * @param sections Section of LED strip
   * @param color Color for pattern
   */
  private void strobe(Color color, Section... sections) {
    boolean on = ((Timer.getFPGATimestamp() % STROBE_DURATION) / STROBE_DURATION) > 0.5;
    solid(on ? color : Color.kBlack, sections);
  }

  /**
   * Set LED strip section to breathe pattern
   * @param sections Section of LED strip
   * @param color Color for pattern
   */
  private void breathe(Color color, Section... sections) {
    double x = ((Timer.getFPGATimestamp() % BREATHE_DURATION) / BREATHE_DURATION) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (color.red * (1 - ratio));
    double green = (color.green * (1 - ratio));
    double blue = (color.blue * (1 - ratio));
    solid(new Color(red, green, blue), sections);
  }

  /**
   * Set LED strip section to wave pattern
   * @param section Section of LED strip
   * @param color Color for pattern
   */
  private void wave(Color color, Section... sections) {
    double x = (1 - ((Timer.getFPGATimestamp() % WAVE_DURATION) / WAVE_DURATION)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / WAVE_CYCLE_LENGTH;
    for (int i = 0; i < Section.end(m_buffer, sections); i++) {
      x += xDiffPerLed;
      if (i >= Section.start(m_buffer, sections)) {
        double ratio = (Math.pow(Math.sin(x), WAVE_EXPONENT) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) ratio = (-Math.pow(Math.sin(x + Math.PI), WAVE_EXPONENT) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) ratio = 0.5;
        double red = (color.red * (1 - ratio));
        double green = (color.green * (1 - ratio));
        double blue = (color.blue * (1 - ratio));
        if (Section.contains(i, m_buffer, sections)) m_buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  /**
   * Set LED strip section to rainbow
   * @param sections Section of LED strip
   */
  private void rainbow(Section... sections) {
    double x = (1 - ((Timer.getFPGATimestamp() / RAINBOW_DURATION) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / RAINBOW_CYCLE_LENGTH;
    for (int i = 0; i < Section.end(m_buffer, sections); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (Section.contains(i, m_buffer, sections)) m_buffer.setHSV(i, (int)x, 255, 255);
    }
  }

  /**
   * Set pattern of LED strip section
   * @param section LED strip ection to set
   * @param pattern Desired pattern
   * @param color Desired color
   */
  private void setPattern(Pattern pattern, Section... sections) {
    switch (pattern.type) {
      case SOLID:
        solid(pattern.color, sections);
        break;
      case STROBE:
        strobe(pattern.color, sections);
        break;
      case BREATHE:
        breathe(pattern.color, sections);
        break;
      case WAVE:
        wave(pattern.color, sections);
        break;
      case RAINBOW:
        rainbow(sections);
        break;
      default:
        off(sections);
        break;
    }
  }

  /**
   * Set pattern and color of LED strip
   * @param pattern Desired pattern
   */
  public void set(Pattern pattern) {
    set(pattern, Section.FULL);
  }

  /**
   * Set pattern and color of LED strip sections
   * @param pattern Desired pattern
   * @param sections LED strip sections to set
   */
  public void set(Pattern pattern, Section... sections) {
    // Remove all conflicting scheduled LED patterns
    m_sectionLEDPatterns.entrySet().removeIf(
      (entry) -> !Collections.disjoint(Arrays.asList(entry.getKey()), Arrays.asList(sections))
    );

    // Schedule LED pattern
    m_sectionLEDPatterns.put(sections, pattern);
  }

  /**
   * Update LED strip pattern
   */
  protected void update() {
    m_sectionLEDPatterns.forEach(
      (sections, pattern) -> setPattern(pattern, sections)
    );
    m_leds.setData(m_buffer);
  }

  /**
   * Prepare for LED override
   */
  protected void startOverride() {
    // Save LED patterns
    m_tempLEDPatterns.clear();
    m_tempLEDPatterns.putAll(m_sectionLEDPatterns);
  }

  /**
   * Restore LED patterns after override
   */
  protected void endOverride() {
    m_sectionLEDPatterns.clear();
    m_sectionLEDPatterns.putAll(m_tempLEDPatterns);
  }

  /**
   * Turn off LED strip  
   */
  public void off() {
    set(Pattern.OFF, Section.FULL);
  }

  /**
   * Turn off LED strip sections
   * @param section LED strip section
   */
  public void off(Section... sections) {
    set(Pattern.OFF, sections);
  }

  @Override
  public void close() {
    m_leds.close();
  }
}
