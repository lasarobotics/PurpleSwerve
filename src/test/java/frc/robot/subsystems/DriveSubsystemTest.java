// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants;
import frc.robot.utils.MAXSwerveModule;
import frc.robot.utils.MAXSwerveModule.ModuleLocation;
import frc.robot.utils.SparkMax;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class DriveSubsystemTest {
  private final double DELTA = 5e-3;
  private final boolean MOCK_HARDWARE = false;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private SparkMax m_lFrontDriveMotor, m_lFrontRotateMotor;
  private SparkMax m_rFrontDriveMotor, m_rFrontRotateMotor;
  private SparkMax m_lRearDriveMotor, m_lRearRotateMotor;
  private SparkMax m_rRearDriveMotor, m_rRearRotateMotor;

  private AbsoluteEncoder m_lFrontRotateEncoder;
  private AbsoluteEncoder m_rFrontRotateEncoder;
  private AbsoluteEncoder m_lRearRotateEncoder;
  private AbsoluteEncoder m_rRearRotateEncoder;
  
  private AHRS m_navx;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_lFrontDriveMotor = mock(SparkMax.class);
    m_lFrontRotateMotor = mock(SparkMax.class);
    m_lFrontRotateEncoder = mock(AbsoluteEncoder.class);
    m_rFrontDriveMotor = mock(SparkMax.class);
    m_rFrontRotateMotor = mock(SparkMax.class);
    m_rFrontRotateEncoder = mock(AbsoluteEncoder.class);
    m_lRearDriveMotor = mock(SparkMax.class);
    m_lRearRotateMotor = mock(SparkMax.class);
    m_lRearRotateEncoder = mock(AbsoluteEncoder.class);
    m_rRearDriveMotor = mock(SparkMax.class);
    m_rRearRotateMotor = mock(SparkMax.class);
    m_rRearRotateEncoder = mock(AbsoluteEncoder.class);
    m_navx = mock(AHRS.class);

    // Create hardware object using mock devices
    m_drivetrainHardware = new DriveSubsystem.Hardware(
      MOCK_HARDWARE,
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_lFrontDriveMotor, m_lFrontRotateMotor, m_lFrontRotateEncoder),
        ModuleLocation.LeftFront, 
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_rFrontDriveMotor, m_rFrontRotateMotor, m_rFrontRotateEncoder),
        ModuleLocation.RightFront, 
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_lRearDriveMotor, m_lRearRotateMotor, m_lRearRotateEncoder),
        ModuleLocation.LeftRear, 
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_rRearDriveMotor, m_rRearRotateMotor, m_rRearRotateEncoder),
        ModuleLocation.RightRear,
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      m_navx
    );

    // Create DriveSubsystem object
    m_driveSubsystem = new DriveSubsystem(
      m_drivetrainHardware,
      Constants.Drive.DRIVE_TURN_PID.kP,
      Constants.Drive.DRIVE_TURN_PID.kD,
      Constants.Drive.DRIVE_TURN_SCALAR,
      Constants.HID.CONTROLLER_DEADBAND,
      Constants.Drive.DRIVE_LOOKAHEAD,
      Constants.Drive.DRIVE_SLIP_RATIO,
      Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_INPUT_CURVE
    );
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can drive forward")
  public void forward() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(0.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED);

    // Try to drive forward
    m_driveSubsystem.teleopPID(+1.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can drive in reverse")
  public void reverse() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(0.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED);

    // Try to drive in reverse
    m_driveSubsystem.teleopPID(-1.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can strafe left")
  public void strafeLeft() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(0.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to strafe left
    m_driveSubsystem.teleopPID(0.0, +1.0, 0.0);

    // Verify motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can strafe right")
  public void strafeRight() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(0.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to strafe right
    m_driveSubsystem.teleopPID(0.0, -1.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can rotate left")
  public void rotateLeft() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(90.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to rotate left
    m_driveSubsystem.teleopPID(0.0, 0.0, +1.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can rotate right")
  public void rotateRight() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(90.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to rotate right
    m_driveSubsystem.teleopPID(0.0, 0.0, -1.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(7)
  @DisplayName("Test if robot can stop")
  public void stop() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(0.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to stop
    m_driveSubsystem.teleopPID(0.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(8)
  @DisplayName("Test if robot can maintain orientation")
  public void maintainOrientation() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(0.0);
    when(m_navx.getAngle()).thenReturn(+30.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to stay still
    m_driveSubsystem.teleopPID(0.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(9)
  @DisplayName("Test if robot can limit wheel slip")
  public void tractionControl() {
    // Hardcode sensor values
    when(m_navx.getRate()).thenReturn(0.0);
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    when(m_lFrontDriveMotor.getEncoderVelocity()).thenReturn(-1.0);
    when(m_rFrontDriveMotor.getEncoderVelocity()).thenReturn(-1.0);
    when(m_lRearDriveMotor.getEncoderVelocity()).thenReturn(+1.0);
    when(m_rRearDriveMotor.getEncoderVelocity()).thenReturn(-1.0);

    // Try to drive forward
    m_driveSubsystem.teleopPID(+1.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }
}
