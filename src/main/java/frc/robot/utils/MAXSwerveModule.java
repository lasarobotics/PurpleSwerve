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

/** Add your docs here. */
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
    LeftFront(0, +45.0),
    RightFront(1, -45.0),
    LeftRear(2, -45.0),
    RightRear(3, +45.0);

    public final int value;
    public final double lockPosition;
    private ModuleLocation(int value, double lockPosition) {
      this.value = value;
      this.lockPosition = lockPosition;
    }
  }

  private final double MAX_VOLTAGE = 12.0;

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
   * @param location Location of module relative to center of robot
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
      double rotateConversionFactor = 360.0;
      m_rotateEncoder.setPositionConversionFactor(rotateConversionFactor);
      m_rotateEncoder.setVelocityConversionFactor(rotateConversionFactor / 60);

      m_rotateMotor.getPIDController().setPositionPIDWrappingEnabled(true);
      m_rotateMotor.getPIDController().setPositionPIDWrappingMinInput(0.0);
      m_rotateMotor.getPIDController().setPositionPIDWrappingMaxInput(360.0);
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
    // Optimize swerve module rotation state
    // CANCoder returns an angle in degrees
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(m_rotateEncoder.getPosition()));

    // Set rotate motor position
    m_rotateMotor.set(state.angle.getDegrees(), ControlType.kPosition);
    
    // Set drive motor speed
    m_driveMotor.set(state.speedMetersPerSecond, ControlType.kVelocity);
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
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(m_rotateEncoder.getPosition()));
  }

  /**
   * Get module position
   * @return Current module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getEncoderPosition(), Rotation2d.fromDegrees(m_rotateEncoder.getPosition()));
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
    m_rotateMotor.set(m_location.lockPosition, ControlType.kPosition);
    m_driveMotor.stopMotor();
  }

  /**
   * Reset swerve module to 0 degrees
   */
  public void reset() {
    m_rotateMotor.set(0.0, ControlType.kPosition);
    m_driveMotor.stopMotor();
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
