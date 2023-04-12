package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

public class SparkMax extends CANSparkMax {
  public static final int PID_SLOT = 0;
  private static final int ALT_ENCODER_CPR = 8192;

  /**
   * Create a Spark Max object that is unit-testing friendly
   * @param deviceID The device ID
   * @param motorType The motor type connected to the controller
   */
  public SparkMax(int deviceID, MotorType motorType) {
    super(deviceID, motorType);
  }

  /**
   * Set motor output value
   * @param value Value to set
   * @param ctrl Desired control mode
   */
  public void set(double value, ControlType ctrl) {
    getPIDController().setReference(value, ctrl);
  }

  /**
   * Set motor output value with arbitrary feed forward
   * @param value Value to set
   * @param ctrl Desired control mode
   * @param arbFeedforward Feed forward value
   * @param arbFFUnits Feed forward units
   */
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits) {
    getPIDController().setReference(value, ctrl, PID_SLOT, arbFeedforward, arbFFUnits);
  }


  /**
   * Set motor output value with arbitrary feed forward
   * @param value Value to set
   * @param ctrl Desired control mode
   * @param arbFeedforward Feed forward value
   * @param arbFFUnits Feed forward units
   */
  public void set(double value, ControlType ctrl, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits, int pidSlot) {
    getPIDController().setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
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
    return getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
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
    return getRelativeEncoder().getPosition();
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   * @return Number the RPM of the motor
   */
  public double getRelativeEncoderVelocity() {
    return getRelativeEncoder().getVelocity();
  }

  /**
   * Reset relative encoder
   */
  public void resetRelativeEncoder() {
    getRelativeEncoder().setPosition(0.0);
  }

  /**
   * Reset NEO built-in encoder
   */
  public void resetEncoder() {
    getEncoder().setPosition(0.0);
  }
}
