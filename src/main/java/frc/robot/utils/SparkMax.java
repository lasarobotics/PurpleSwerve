package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;

public class SparkMax {

  public static class ID {
    public final int deviceID;
    public final String name;

    public ID(int deviceID, String name) {
      this.deviceID = deviceID;
      this.name = name;
    }
  }

  @AutoLog
  public static class SparkMaxInputs {
    public double encoderPosition = 0.0;
    public double encoderVelocity = 0.0;
    public double analogPosition = 0.0;
    public double analogVelocity = 0.0;
    public double absoluteEncoderPosition = 0.0;
    public double absoluteEncoderVelocity = 0.0;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
  }

  private static final int PID_SLOT = 0;

  private CANSparkMax m_motor;
  private final SparkMaxInputsAutoLogged m_inputs;
  private String m_name;

  /**
   * Create a Spark Max object that is unit-testing friendly
   * @param deviceID The device ID
   * @param motorType The motor type connected to the controller
   */
  public SparkMax(ID id, MotorType motorType) {
    this.m_motor = new CANSparkMax(id.deviceID, motorType);
    this.m_inputs = new SparkMaxInputsAutoLogged();
    this.m_name = id.name;
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
    m_motor.getPIDController().setReference(value, ctrl);
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
    m_motor.getPIDController().setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
  }

  /**
   * Sets the idle mode setting for the SPARK MAX.
   * @param mode Idle mode (coast or brake).
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIdleMode(IdleMode mode) {
    return m_motor.setIdleMode(mode);
  }

  /**
   * Sets the current limit in Amps.
   *
   * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
   * limit. This limit is enabled by default and used for brushless only. This limit is highly
   * recommended when using the NEO brushless motor.
   *
   * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
   * that could be enough to cause damage to the motor and controller. This current limit provides a
   * smarter strategy to deal with high current draws and keep the motor and controller operating in
   * a safe region.
   *
   * @param limit The current limit in Amps.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSmartCurrentLimit(int limit) {
    return m_motor.setSmartCurrentLimit(limit);
  }

  /**
   * Sets the voltage compensation setting for all modes on the SPARK MAX and enables voltage
   * compensation.
   * @param nominalVoltage Nominal voltage to compensate output to
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError enableVoltageCompensation(double nominalVoltage) {
    return m_motor.enableVoltageCompensation(nominalVoltage);
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs() {
    m_inputs.encoderPosition = getEncoderPosition();
    m_inputs.encoderVelocity = getEncoderVelocity();
    m_inputs.analogPosition = m_motor.getAnalog(Mode.kAbsolute).getPosition();
    m_inputs.analogVelocity = m_motor.getAnalog(Mode.kAbsolute).getVelocity();
    m_inputs.absoluteEncoderPosition = getAbsoluteEncoderPosition();
    m_inputs.absoluteEncoderVelocity = getAbsoluteEncoderVelocity();
    m_inputs.forwardLimitSwitch = m_motor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
    m_inputs.reverseLimitSwitch = m_motor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }
  
  public void periodic() {
    updateInputs();
    Logger.getInstance().processInputs(m_name, m_inputs);
  }

  /**
   * Returns an object for interfacing with the hall sensor integrated into a brushless motor, which
   * is connected to the front port of the SPARK MAX.
   *
   * <p>To access a quadrature encoder connected to the encoder pins or the front port of the SPARK
   * MAX, you must call the version of this method with EncoderType and countsPerRev parameters.
   *
   * @return An object for interfacing with the integrated encoder.
   */
  public RelativeEncoder getEncoder() {
    return m_motor.getEncoder();
  }

  /**
   * Get the position of the motor. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().W
   * @return Number of rotations of the motor
   */
  public double getEncoderPosition() {
    return getEncoder().getPosition();
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  public double getEncoderVelocity() {
    return getEncoder().getVelocity();
  }

  /**
   * Returns an object for interfacing with a connected absolute encoder.
   * @return An object for interfacing with a connected absolute encoder
   */
  public AbsoluteEncoder getAbsoluteEncoder() {
    return m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  }

  /**
   * Get position of the motor. This returns the native units 'rotations' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   * @return Number of rotations of the motor
   */
  public double getAbsoluteEncoderPosition() {
    return getAbsoluteEncoder().getPosition();
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  public double getAbsoluteEncoderVelocity() {
    return getAbsoluteEncoder().getVelocity();
  }

  /** @return An object for interfacing with the integrated PID controller. */
  public SparkMaxPIDController getPIDController() {
    return m_motor.getPIDController();
  }

  /**
   * Get the underlying CANSparkMax motor
   * @return Underlying CANSparkMax motor
   */
  public CANSparkMax getMotor() {
    return m_motor;
  }

  /**
   * Reset NEO built-in encoder
   */
  public void resetEncoder() {
    getEncoder().setPosition(0.0);
  }

  /**
   * Restore motor controller parameters to factory default until the next controller reboot
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError restoreFactoryDefaults() {
    return m_motor.restoreFactoryDefaults();
  }

  /** 
   * Closes the SPARK MAX Controller 
   */
  public void close() {
    m_motor.close();
  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable 
   * the motor.
   */
  public void stopMotor() {
    m_motor.stopMotor();
  }
}
