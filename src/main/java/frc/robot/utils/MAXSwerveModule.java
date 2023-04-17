// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** 
 * MAXSwerve Module
 */
public class MAXSwerveModule implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax driveMotor;
    private SparkMax rotateMotor;
    private AbsoluteEncoder rotateEncoder;

    public Hardware(boolean isHardwareReal, SparkMax driveMotor, SparkMax rotateMotor, AbsoluteEncoder rotateEncoder) {
      this.isHardwareReal = isHardwareReal;
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
      this.rotateEncoder = rotateEncoder;
    }
  }

  public enum ModuleLocation {
    LeftFront(0, -Math.PI / 2),
    RightFront(1, +0.0),
    LeftRear(2, +Math.PI),
    RightRear(3, +Math.PI / 2);

    public final int value;
    public final double offset;
    private ModuleLocation(int value, double offset) {
      this.value = value;
      this.offset = offset;
    }
  }

  private final double MAX_VOLTAGE = 12.0;
  private final double LOCK_POSITION = +Math.PI / 4;

  private final SparkMax m_driveMotor;
  private final SparkMax m_rotateMotor;
  private final AbsoluteEncoder m_rotateEncoder;
  private final SparkPIDConfig m_driveMotorConfig;
  private final SparkPIDConfig m_rotateMotorConfig;
  private final Translation2d m_moduleCoordinate;
  private final ModuleLocation m_location;

  private final double m_driveGearRatio;
  private final double m_driveWheelDiameter;
  private final double m_radius;

  /**
   * Create an instance of a MAXSwerveModule
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Location of module
   * @param driveMotorConfig Drive motor velocity PID config
   * @param rotateMotorConfig Rotate motor position PID config
   * @param wheelbase Robot wheelbase in meters
   * @param trackWidth Robot track width in meters
   * @param driveGearRatio Drive gear ratio
   * @param driveWheelDiameter Wheel diameter in meters
   */
  public MAXSwerveModule(Hardware swerveHardware, ModuleLocation location, 
                         SparkPIDConfig driveMotorConfig, SparkPIDConfig rotateMotorConfig,
                         double wheelbase, double trackWidth, double driveGearRatio, double driveWheelDiameter) {
    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_rotateEncoder = swerveHardware.rotateEncoder;
    this.m_location = location;
    this.m_driveMotorConfig = driveMotorConfig;
    this.m_rotateMotorConfig = rotateMotorConfig;
    this.m_driveGearRatio = driveGearRatio;
    this.m_driveWheelDiameter = driveWheelDiameter;

    // Reset devices to default
    m_driveMotor.restoreFactoryDefaults();
    m_rotateMotor.restoreFactoryDefaults();

    // Set drive motor to coast
    m_driveMotor.setIdleMode(IdleMode.kCoast);

    // Set rotate motor to brake
    m_rotateMotor.setIdleMode(IdleMode.kBrake);

    // Enable voltage compensation
    m_driveMotor.enableVoltageCompensation(MAX_VOLTAGE);
    m_rotateMotor.enableVoltageCompensation(MAX_VOLTAGE);

    // Only do this stuff if hardware is real
    if (swerveHardware.isHardwareReal) {
      // Initialize PID
      m_driveMotorConfig.initializeSparkPID(m_driveMotor, m_driveMotor.getEncoder());
      m_rotateMotorConfig.initializeSparkPID(m_rotateMotor, m_rotateEncoder);

      // Set drive encoder conversion factor
      double driveConversionFactor = m_driveWheelDiameter * Math.PI / m_driveGearRatio;
      m_driveMotor.getEncoder().setPositionConversionFactor(driveConversionFactor);
      m_driveMotor.getEncoder().setVelocityConversionFactor(driveConversionFactor / 60);

      // Set rotate encoder conversion factor
      double rotateConversionFactor = 2 * Math.PI;
      m_rotateEncoder.setPositionConversionFactor(rotateConversionFactor);
      m_rotateEncoder.setVelocityConversionFactor(rotateConversionFactor / 60);

      // Enable PID wrapping
      m_rotateMotor.getPIDController().setPositionPIDWrappingEnabled(true);
      m_rotateMotor.getPIDController().setPositionPIDWrappingMinInput(0.0);
      m_rotateMotor.getPIDController().setPositionPIDWrappingMaxInput(2 * Math.PI);
    }

    // Calculate module coordinate
    switch (location) {
      case LeftFront:
        m_moduleCoordinate = new Translation2d(+wheelbase / 2, +trackWidth / 2);
        break;
      case RightFront:
        m_moduleCoordinate = new Translation2d(+wheelbase / 2, -trackWidth / 2);
        break;
      case LeftRear:
        m_moduleCoordinate = new Translation2d(-wheelbase / 2, +trackWidth / 2);
        break;
      case RightRear:
        m_moduleCoordinate = new Translation2d(-wheelbase / 2, -trackWidth / 2);
        break;
      default:
        m_moduleCoordinate = new Translation2d();
        break;
    }

    // Get distance from center of robot
    m_radius = m_moduleCoordinate.getNorm();
  }

  public static Hardware initializeHardware(boolean isHardwareReal, int driveMotorID, int rotateMotorID) {
    SparkMax driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    SparkMax rotateMotor = new SparkMax(rotateMotorID, MotorType.kBrushless);
    Hardware swerveModuleHardware = new Hardware(isHardwareReal, driveMotor, rotateMotor, rotateMotor.getAbsoluteEncoder());

    return swerveModuleHardware;
  }

  /**
   * Set swerve module direction and speed
   * @param state Desired swerve module state
   */
  public void set(SwerveModuleState state) {
    // Apply chassis angular offset to the requested state.
    SwerveModuleState desiredState = new SwerveModuleState(
      state.speedMetersPerSecond,
      state.angle.plus(Rotation2d.fromRadians(m_location.offset))
    );

    // Optimize swerve module rotation state
    // CANCoder returns an angle in radians
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(m_rotateEncoder.getPosition()));

    // Set rotate motor position
    m_rotateMotor.set(desiredState.angle.getRadians(), ControlType.kPosition);
    
    // Set drive motor speed
    m_driveMotor.set(desiredState.speedMetersPerSecond, ControlType.kVelocity);
  }

  /**
   * Set swerve module direction and speed
   * @param states array of states for all swerve modules
   */
  public void set(SwerveModuleState[] states) {
    set(states[m_location.value]);
  }

  /**
   * Get velocity of drive wheel
   * @return velocity of drive wheel in m/s
   */
  public double getDriveVelocity() {
    return m_driveMotor.getEncoderVelocity();
  }

  /**
   * Get current module state
   * @return Current module state
   */
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateEncoder.getPosition() - m_location.offset)
    );
  }

  /**
   * Get module position
   * @return Current module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveMotor.getEncoderPosition(),
      Rotation2d.fromRadians(m_rotateEncoder.getPosition() - m_location.offset)
    );
  }

  /**
   * Get real speed of module
   * @param inertialVelocity Inertial velocity of robot in m/s
   * @param turnRate Turn rate of robot in degrees per second
   * @return Speed of module in m/s
   */
  public double calculateRealSpeed(double inertialVelocity, double turnRate) {
    return inertialVelocity + Math.toRadians(turnRate) * m_radius;
  }

  /**
   * Reset drive motor encoder
   */
  public void resetDriveEncoder() {
    m_driveMotor.resetEncoder();
  }

  /**
   * Lock swerve module
   */
  public void lock() {
    set(new SwerveModuleState(0.0, Rotation2d.fromRadians(LOCK_POSITION)));
  }

  /**
   * Reset swerve module to 0 degrees
   */
  public void reset() {
    set(new SwerveModuleState(0.0, Rotation2d.fromRadians(0.0)));
  }

  /**
   * Get coordinate of module relative to the center of the robot
   * @return X, Y coordinate in meters
   */
  public Translation2d getModuleCoordinate() {
    return m_moduleCoordinate;
  }

  /**
   * Stop swerve module
   */
  public void stop() {
    m_rotateMotor.stopMotor();
    m_driveMotor.stopMotor();
  }

  @Override
  public void close() {
    m_driveMotor.close();
    m_rotateMotor.close();
  }
}
