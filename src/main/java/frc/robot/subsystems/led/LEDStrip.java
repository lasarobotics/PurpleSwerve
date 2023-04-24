// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/** LED Strip */
public class LEDStrip {
  public static class Hardware {
    AddressableLED ledStrip;

    public Hardware(AddressableLED ledStrip) {
      this.ledStrip = ledStrip;
    }
  }

  /**
   * Valid LED patterns
   */
  public enum Pattern {
    SOLID, STROBE, BREATHE, WAVE, RAINBOW;
  }

  /**
   * LED strip sections
   */
  private static enum Section {
    START,
    MIDDLE,
    END,
    FULL;

    private static final int SMALL_SECTION_LENGTH = 10;

    private int start(AddressableLEDBuffer buffer) {
      switch (this) {
        case START:
          return 0;
        case MIDDLE:
          return SMALL_SECTION_LENGTH;
        case END:
          return buffer.getLength() - SMALL_SECTION_LENGTH;
        case FULL:
          return 0;
        default:
          return 0;
      }
    }

    private int end(AddressableLEDBuffer buffer) {
      switch (this) {
        case START:
          return SMALL_SECTION_LENGTH;
        case MIDDLE:
          return buffer.getLength() - SMALL_SECTION_LENGTH;
        case END:
          return buffer.getLength();
        case FULL:
          return buffer.getLength();
        default:
          return buffer.getLength();
      }
    }
  }

  private AddressableLED m_leds;
  private AddressableLEDBuffer m_buffer;
  private Runnable m_sectionLEDPatterns[];

  private static final double STROBE_DURATION = 0.1;
  private static final double BREATHE_DURATION = 1.0;
  private static final double RAINBOW_CYCLE_LENGTH = 25.0;
  private static final double RAINBOW_DURATION = 0.25;
  private static final double WAVE_EXPONENT = 0.4;
  private static final double WAVE_CYCLE_LENGTH = 25.0;
  private static final double WAVE_DURATION = 1.0;

  private static final Color TEAM_COLOR = Color.kPurple;
  private static final Color ALLIANCE_COLORS[] = { Color.kRed, Color.kBlue, TEAM_COLOR };
  private static final Runnable DO_NOTHING = () -> {};

  /**
   * Create an instance of an LED strip
   * @param ledStripHardware Hardware devices required by LED strip
   * @param length Number of LEDs in strip
   */
  public LEDStrip(Hardware ledStripHardware, int length) {
    this.m_leds = ledStripHardware.ledStrip;
    this.m_buffer = new AddressableLEDBuffer(length);
    this.m_sectionLEDPatterns = new Runnable[] {
      () -> off(Section.FULL),
      DO_NOTHING,
      DO_NOTHING,
      DO_NOTHING
    };
  }

  /**
   * Initialize hardware devices for LED strip
   * 
   * @param ledStripPort PWM port for LED strip
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware(int ledStripPort) {
    Hardware ledStripHardware = new Hardware(new AddressableLED(ledStripPort));

    return ledStripHardware;
  }

  /**
   * Set LED strip section to solid color
   * @param section Section of LED strip
   * @param color Color to set
   */
  private void solid(Section section, Color color) {
    for (int i = section.start(m_buffer); i < section.end(m_buffer); i++) 
      m_buffer.setLED(i, color);
  }

  /**
   * Set LED strip section to strobe pattern
   * @param section Section of LED strip
   * @param color Color for pattern
   */
  private void strobe(Section section, Color color) {
    boolean on = ((Timer.getFPGATimestamp() % STROBE_DURATION) / STROBE_DURATION) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  /**
   * Set LED strip section to breathe pattern
   * @param section Section of LED strip
   * @param color Color for pattern
   */
  private void breathe(Section section, Color color) {
    double x = ((Timer.getFPGATimestamp() % BREATHE_DURATION) / BREATHE_DURATION) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (color.red * (1 - ratio));
    double green = (color.green * (1 - ratio));
    double blue = (color.blue * (1 - ratio));
    solid(section, new Color(red, green, blue));
  }

  /**
   * Set LED strip section to wave pattern
   * @param section Section of LED strip
   * @param color Color for pattern
   */
  private void wave(Section section, Color color) {
    double x = (1 - ((Timer.getFPGATimestamp() % WAVE_DURATION) / WAVE_DURATION)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / WAVE_CYCLE_LENGTH;
    for (int i = 0; i < section.end(m_buffer); i++) {
      x += xDiffPerLed;
      if (i >= section.start(m_buffer)) {
        double ratio = (Math.pow(Math.sin(x), WAVE_EXPONENT) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) ratio = (-Math.pow(Math.sin(x + Math.PI), WAVE_EXPONENT) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) ratio = 0.5;
        double red = (color.red * (1 - ratio));
        double green = (color.green * (1 - ratio));
        double blue = (color.blue * (1 - ratio));
        m_buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  /**
   * Set LED strip section to rainbow
   * @param section Section of LED strip
   */
  private void rainbow(Section section) {
    double x = (1 - ((Timer.getFPGATimestamp() / RAINBOW_DURATION) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / RAINBOW_CYCLE_LENGTH;
    for (int i = 0; i < section.end(m_buffer); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start(m_buffer)) m_buffer.setHSV(i, (int)x, 255, 255);
    }
  }

  /**
   * Set pattern of LED strip section
   * @param section LED strip ection to set
   * @param pattern Desired pattern
   * @param color Desired color
   */
  private void setPattern(Section section, Pattern pattern, Color color) {
    switch (pattern) {
      case SOLID:
        solid(section, color);
        break;
      case STROBE:
        strobe(section, color);
        break;
      case BREATHE:
        breathe(section, color);
        break;
      case WAVE:
        wave(section, color);
        break;
      case RAINBOW:
        rainbow(section);
        break;
      default:
        off(section);
        break;
    }
  }

  /**
   * Set pattern and color of LED strip section
   * @param section LED strip section to set
   * @param pattern Desired pattern
   * @param color Desired color
   */
  public void set(Section section, Pattern pattern, Color color) {
    if (section.equals(Section.FULL)) {
      m_sectionLEDPatterns[Section.START.ordinal()] = DO_NOTHING;
      m_sectionLEDPatterns[Section.MIDDLE.ordinal()] = DO_NOTHING;
      m_sectionLEDPatterns[Section.END.ordinal()] = DO_NOTHING;
    } else m_sectionLEDPatterns[Section.FULL.ordinal()] = DO_NOTHING;

    m_sectionLEDPatterns[section.ordinal()] = () -> { setPattern(section, pattern, color); };
  }

  /**
   * Update LED strip colors/patterns
   */
  protected void update() {
    for (Runnable sectionLEDPattern : m_sectionLEDPatterns) sectionLEDPattern.run();
    m_leds.setData(m_buffer);
  }

  /**
   * Turn off LED strip  
   */
  public void off() {
    set(Section.FULL, Pattern.SOLID, Color.kBlack);
  }

  /**
   * Turn off LED strip section
   * @param section LED strip section
   */
  public void off(Section section) {
    set(section, Pattern.SOLID, Color.kBlack);
  }

  /**
   * Set LED strip to alliance color
   * @param pattern Desired pattern
   */
  public void setAllianceColor(Pattern pattern) {
    setAllianceColor(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to Alliance color
   * @param section LED strip section
   * @param pattern Desired pattern
   */
  public void setAllianceColor(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, ALLIANCE_COLORS[DriverStation.getAlliance().ordinal()]);
  }

  /**
   * Set LED strip to team color
   * @param pattern Desired pattern
   */
  public void setTeamColor(Pattern pattern) {
    setTeamColor(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to team color
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setTeamColor(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, TEAM_COLOR);
  }

  /**
   * Set LED strip to white
   * @param pattern Desired pattern
   */
  public void setWhite(Pattern pattern) {
    setWhite(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to white
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setWhite(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kWhite);
  }

  /**
   * Set LED strip to red
   * @param pattern Desired pattern
   */
  public void setRed(Pattern pattern) {
    setRed(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to red
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setRed(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kRed);
  }

  /**
   * Set LED strip to orange
   * @param pattern Desired pattern
   */
  public void setOrange(Pattern pattern) {
    setOrange(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to yellow
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setOrange(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kOrange);
  }

  /**
   * Set LED strip to yellow
   * @param pattern Desired pattern
   */
  public void setYellow(Pattern pattern) {
    setYellow(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to yellow
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setYellow(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kYellow);
  }

  /**
   * Set LED strip to green
   * @param pattern Desired pattern
   */
  public void setGreen(Pattern pattern) {
    setGreen(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to green
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setGreen(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kGreen);
  }

  /**
   * Set LED strip to blue
   * @param pattern Desired pattern
   */
  public void setBlue(Pattern pattern) {
    setBlue(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to blue
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setBlue(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kBlue);
  }

  /**
   * Set LED strip to indigo
   * @param pattern Desired pattern
   */
  public void setIndigo(Pattern pattern) {
    setIndigo(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to indigo
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setIndigo(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kIndigo);
  }

  /**
   * Set LED strip to violet
   * @param pattern Desired pattern
   */
  public void setViolet(Pattern pattern) {
    setViolet(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to violet
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setViolet(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kViolet);
  }

  /**
   * Set LED strip to pink
   * @param pattern Desired pattern
   */
  public void setPink(Pattern pattern) {
    setPink(Section.FULL, pattern);
  }

  /**
   * Set LED strip section to pink
   * @param section LED strip section
   * @param pattern Desired pattern 
   */
  public void setPink(Section section, Pattern pattern) {
    if (pattern.equals(Pattern.RAINBOW)) pattern = Pattern.SOLID;
    set(section, pattern, Color.kPink);
  }
}
