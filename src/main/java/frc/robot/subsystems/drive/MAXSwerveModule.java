// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

/** 
 * MAXSwerve Module
 */
public class MAXSwerveModule implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax driveMotor;
    private SparkMax rotateMotor;

    public Hardware(boolean isHardwareReal, SparkMax driveMotor, SparkMax rotateMotor) {
      this.isHardwareReal = isHardwareReal;
      this.driveMotor = driveMotor;
      this.rotateMotor = rotateMotor;
    }
  }

  public enum ModuleLocation {
    LeftFront(0, -Math.PI / 2),
    RightFront(1, +0.0),
    LeftRear(2, +Math.PI),
    RightRear(3, +Math.PI / 2);

    public final int index;
    public final double offset;
    private ModuleLocation(int index, double offset) {
      this.index = index;
      this.offset = offset;
    }
  }

  private final double MAX_VOLTAGE = 12.0;
  private final double LOCK_POSITION = Math.PI / 4;
  private final int DRIVE_MOTOR_CURRENT_LIMIT = 50;
  private final int ROTATE_MOTOR_CURRENT_LIMIT = 20;

  private final SparkMax m_driveMotor;
  private final SparkMax m_rotateMotor;
  private final SparkPIDConfig m_driveMotorConfig;
  private final SparkPIDConfig m_rotateMotorConfig;
  private final Translation2d m_moduleCoordinate;
  private final ModuleLocation m_location;

  private final double m_driveGearRatio;
  private final double m_driveWheelDiameter;
  private final double m_radius;

  private static TractionControlController m_tractionControlController;

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

    // Set current limits
    m_driveMotor.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    m_rotateMotor.setSmartCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT);

    // Only do this stuff if hardware is real
    if (swerveHardware.isHardwareReal) {
      // Initialize PID
      m_driveMotorConfig.initializeSparkPID(m_driveMotor, m_driveMotor.getEncoder());
      m_rotateMotorConfig.initializeSparkPID(m_rotateMotor, m_rotateMotor.getAbsoluteEncoder());

      // Set drive encoder conversion factor
      double driveConversionFactor = m_driveWheelDiameter * Math.PI / m_driveGearRatio;
      m_driveMotor.getEncoder().setPositionConversionFactor(driveConversionFactor);
      m_driveMotor.getEncoder().setVelocityConversionFactor(driveConversionFactor / 60);

      // Set rotate encoder conversion factor
      double rotateConversionFactor = 2 * Math.PI;
      m_rotateMotor.getAbsoluteEncoder().setPositionConversionFactor(rotateConversionFactor);
      m_rotateMotor.getAbsoluteEncoder().setVelocityConversionFactor(rotateConversionFactor / 60);

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

  /**
   * Initialize hardware devices for MAXSwerve module
   * @param isHardwareReal True if hardware is real
   * @param driveMotorID Drive motor ID
   * @param rotateMotorID Rotate motor ID
   * @return Hardware object containing all necessary objects for a MAXSwerve module
   */
  public static Hardware initializeHardware(boolean isHardwareReal, SparkMax.ID driveMotorID, SparkMax.ID rotateMotorID) {
    Hardware swerveModuleHardware = new Hardware(
      isHardwareReal, 
      new SparkMax(driveMotorID, MotorType.kBrushless),
      new SparkMax(rotateMotorID, MotorType.kBrushless)
    );

    return swerveModuleHardware;
  }

  /**
   * Set traction control controller to use
   * @param tractionControlController
   */
  public static void setTractionControlController(TractionControlController tractionControlController) {
    m_tractionControlController = tractionControlController;
  }

  /**
   * Get real speed of module
   * @param inertialVelocity Inertial velocity of robot (m/s)
   * @param turnRate Turn rate of robot (degrees/s)
   * @return Speed of module (m/s)
   */
  private double calculateRealSpeed(double inertialVelocity, double turnRate) {
    return inertialVelocity + Math.toRadians(turnRate) * m_radius;
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
    // REV encoder returns an angle in radians
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(m_rotateMotor.getAbsoluteEncoderPosition()));

    // Set rotate motor position
    m_rotateMotor.set(desiredState.angle.getRadians(), ControlType.kPosition);
    
    // Set drive motor speed
    m_driveMotor.set(desiredState.speedMetersPerSecond, ControlType.kVelocity);
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param state Desired swerve module state
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param turnRate Current turn rate (degrees/s)
   */
  public void set(SwerveModuleState state, double inertialVelocity, double turnRate) {
    // Apply traction control
    state.speedMetersPerSecond = m_tractionControlController.calculate(
      state.speedMetersPerSecond,
      calculateRealSpeed(inertialVelocity, turnRate),
      getDriveVelocity()
    );

    // Set swerve module state
    set(state);
  }

  /**
   * Set swerve module direction and speed
   * @param states Array of states for all swerve modules
   */
  public void set(SwerveModuleState[] states) {
    set(states[m_location.index]);
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param states Array of states for all swerve modules
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param turnRate Current turn rate (degrees/s)
   */
  public void set(SwerveModuleState[] states, double inertialVelocity, double turnRate) {
    // Apply traction control
    states[m_location.index].speedMetersPerSecond = m_tractionControlController.calculate(
      states[m_location.index].speedMetersPerSecond,
      calculateRealSpeed(inertialVelocity, turnRate),
      getDriveVelocity()
    );

    // Set swerve module state
    set(states[m_location.index]);
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
      Rotation2d.fromRadians(m_rotateMotor.getAbsoluteEncoderPosition() - m_location.offset)
    );
  }

  /**
   * Get module position
   * @return Current module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveMotor.getEncoderPosition(),
      Rotation2d.fromRadians(m_rotateMotor.getAbsoluteEncoderPosition() - m_location.offset)
    );
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
    set(new SwerveModuleState(0.0, Rotation2d.fromRadians(LOCK_POSITION - m_location.offset)));
  }

  /**
   * Reset swerve module to 0 degrees
   */
  public void reset() {
    set(new SwerveModuleState(0.0, Rotation2d.fromRadians(m_location.offset)));
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
