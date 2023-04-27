package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class SparkMax extends CANSparkMax {

  public static class ID {
    public final int deviceID;
    public final String name;

    public ID(int deviceID, String name) {
      this.deviceID = deviceID;
      this.name = name;
    }
  }

  public static final int PID_SLOT = 0;
  private static final int ALT_ENCODER_CPR = 8192;
  private static final String VALUE_LOG_ENTRY = "Value";
  private static final String CURRENT_LOG_ENTRY = "Current";
  private static final String INTERNAL_ENCODER_POSITION = "Internal encoder position";
  private static final String INTERNAL_ENCODER_VELOCITY = "Internal encoder velocity";
  private static final String RELATIVE_ENCODER_POSITION = "External relative encoder position";
  private static final String RELATIVE_ENCODER_VELOCITY = "External relative encoder velocity";
  private static final String ABSOLUTE_ENCODER_POSITION = "Absolute encoder position";
  private static final String ABSOLUTE_ENCODER_VELOCITY = "Absolute encoder velocity";

  private String m_name;
  private DoubleLogEntry m_valueLogEntry;
  private DoubleLogEntry m_currentLogEntry;
  private DoubleLogEntry m_internalEncoderPositionLogEntry;
  private DoubleLogEntry m_internalEncoderVelocityLogEntry;
  private DoubleLogEntry m_relativeEncoderPositionLogEntry;
  private DoubleLogEntry m_relativeEncoderVelocityLogEntry;
  private DoubleLogEntry m_absoluteEncoderPositionLogEntry;
  private DoubleLogEntry m_absoluteEncoderVelocityLogEntry;

  /**
   * Create a Spark Max object that is unit-testing friendly
   * @param deviceID The device ID
   * @param motorType The motor type connected to the controller
   */
  public SparkMax(ID id, MotorType motorType) {
    super(id.deviceID, motorType);
    this.m_name = id.name;

    DataLog log = DataLogManager.getLog();
    m_valueLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, VALUE_LOG_ENTRY));
    m_currentLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, CURRENT_LOG_ENTRY));
    m_internalEncoderPositionLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, INTERNAL_ENCODER_POSITION));
    m_internalEncoderVelocityLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, INTERNAL_ENCODER_VELOCITY));
    m_relativeEncoderPositionLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, RELATIVE_ENCODER_POSITION));
    m_relativeEncoderVelocityLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, RELATIVE_ENCODER_VELOCITY));
    m_absoluteEncoderPositionLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, ABSOLUTE_ENCODER_POSITION));
    m_absoluteEncoderVelocityLogEntry = new DoubleLogEntry(log, String.join(" ", m_name, ABSOLUTE_ENCODER_VELOCITY));
  }

  /**
   * Log values
   * @param value Value that was set
   * @param ctrl Control mode that was used
   */
  private void log(double value, ControlType ctrl) {
    m_valueLogEntry.setMetadata(ctrl.name());
    m_valueLogEntry.append(value);
    m_currentLogEntry.append(getOutputCurrent());
  }

  /**
   * Set motor output duty cycle
   * @param value Value to set [-1.0, +1.0]
   */
  public void set(double value) {
    set(value, ControlType.kDutyCycle);
  }

  /**
   * Set motor output value
   * @param value Value to set
   * @param ctrl Desired control mode
   */
  public void set(double value, ControlType ctrl) {
    getPIDController().setReference(value, ctrl);
    log(value, ctrl);
  }

  /**
   * Set motor output value with arbitrary feed forward
   * @param value Value to set
   * @param ctrl Desired control mode
   * @param arbFeedforward Feed forward value
   * @param arbFFUnits Feed forward units
   */
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits) {
    set(value, ctrl, arbFeedforward, arbFFUnits, PID_SLOT);
  }


  /**
   * Set motor output value with arbitrary feed forward
   * @param value Value to set
   * @param ctrl Desired control mode
   * @param arbFeedforward Feed forward value
   * @param arbFFUnits Feed forward units
   * @param pidSlot PID slot to use
   */
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits, int pidSlot) {
    getPIDController().setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
    log(value, ctrl);
  }

  /**
   * Get the position of the motor. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().W
   * @return Number of rotations of the motor
   */
  public double getEncoderPosition() {
    double position = getEncoder().getPosition();
    m_internalEncoderPositionLogEntry.append(position);

    return position;
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  public double getEncoderVelocity() {
    double velocity = getEncoder().getVelocity();
    m_internalEncoderVelocityLogEntry.append(velocity);

    return velocity;
  }

  /**
   * Returns an object for interfacing with a connected absolute encoder.
   * @return An object for interfacing with a connected absolute encoder
   */
  public AbsoluteEncoder getAbsoluteEncoder() {
    return getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  }

  /**
   * Get position of the motor. This returns the native units 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  public double getAbsoluteEncoderPosition() {
    double position = getAbsoluteEncoder().getPosition();
    m_absoluteEncoderPositionLogEntry.append(position);
    
    return position;
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  public double getAbsoluteEncoderVelocity() {
    double velocity = getAbsoluteEncoder().getVelocity();
    m_absoluteEncoderVelocityLogEntry.append(velocity);

    return velocity;
  }

  /**
   * Return an object for interfacing with a connected through bore encoder
   * @return An object for interfacing with relative encoder
   */
  public RelativeEncoder getRelativeEncoder() {
    return getAlternateEncoder(ALT_ENCODER_CPR);
  }

  /**
   * Get the position of the motor. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  public double getRelativeEncoderPosition() {
    double position = getRelativeEncoder().getPosition();
    m_relativeEncoderPositionLogEntry.append(position);

    return position;
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  public double getRelativeEncoderVelocity() {
    double velocity = getRelativeEncoder().getVelocity();
    m_relativeEncoderVelocityLogEntry.append(velocity);

    return velocity;
  }

  /**
   * Reset relative encoder
   */
  public void resetRelativeEncoder() {
    getRelativeEncoder().setPosition(0.0);
    DataLogManager.log(m_name + ": External relative encoder reset");
  }

  /**
   * Reset NEO built-in encoder
   */
  public void resetEncoder() {
    getEncoder().setPosition(0.0);
    DataLogManager.log(m_name + ": Internal encoder reset");
  }
}
