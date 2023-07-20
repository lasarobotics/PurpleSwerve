// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
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

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveModule;
import frc.robot.subsystems.drive.MAXSwerveModule.ModuleLocation;
import frc.robot.subsystems.led.LEDStrip;
import frc.robot.utils.NavX2;
import frc.robot.utils.NavX2InputsAutoLogged;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkMaxInputsAutoLogged;
import frc.robot.utils.NavX2.NavX2Inputs;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class AntiTipCommandTest {
  private final double DELTA = 5e-3;
  private final boolean MOCK_HARDWARE = false;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;
  private Command m_antiTipCommand;

  private NavX2 m_navx;

  private SparkMax m_lFrontDriveMotor, m_lFrontRotateMotor;
  private SparkMax m_rFrontDriveMotor, m_rFrontRotateMotor;
  private SparkMax m_lRearDriveMotor, m_lRearRotateMotor;
  private SparkMax m_rRearDriveMotor, m_rRearRotateMotor;

  private LEDStrip m_ledStrip;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_navx = mock(NavX2.class);
    m_lFrontDriveMotor = mock(SparkMax.class);
    m_lFrontRotateMotor = mock(SparkMax.class);
    m_rFrontDriveMotor = mock(SparkMax.class);
    m_rFrontRotateMotor = mock(SparkMax.class);
    m_lRearDriveMotor = mock(SparkMax.class);
    m_lRearRotateMotor = mock(SparkMax.class);
    m_rRearDriveMotor = mock(SparkMax.class);
    m_rRearRotateMotor = mock(SparkMax.class);
    m_ledStrip = mock(LEDStrip.class);

    
    NavX2InputsAutoLogged navxInputs = new NavX2InputsAutoLogged();
    when(m_navx.getInputs()).thenReturn(navxInputs);

    SparkMaxInputsAutoLogged sparkMaxInputs = new SparkMaxInputsAutoLogged();
    
    when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);

    // Create hardware object using mock devices
    m_drivetrainHardware = new DriveSubsystem.Hardware(
      MOCK_HARDWARE,
      m_navx,
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_lFrontDriveMotor, m_lFrontRotateMotor),
        ModuleLocation.LeftFront, 
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_MAX_LINEAR_SPEED,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_rFrontDriveMotor, m_rFrontRotateMotor),
        ModuleLocation.RightFront, 
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_MAX_LINEAR_SPEED,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_lRearDriveMotor, m_lRearRotateMotor),
        ModuleLocation.LeftRear, 
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_MAX_LINEAR_SPEED,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(MOCK_HARDWARE, m_rRearDriveMotor, m_rRearRotateMotor),
        ModuleLocation.RightRear,
        Constants.Drive.DRIVE_VELOCITY_CONFIG,
        Constants.Drive.DRIVE_ROTATE_CONFIG,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_MAX_LINEAR_SPEED,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.DRIVE_GEAR_RATIO,
        DriveSubsystem.DRIVE_GEAR_RATIO
      ),
      m_ledStrip
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

    // Create AntiTipCommand object
    m_antiTipCommand = m_driveSubsystem.ANTI_TIP_COMMAND;
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
    m_antiTipCommand = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can execute anti-tip")
  public void execute() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged(); 
    inputs.rollAngle = +35.0;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to execute anti-tip command
    m_antiTipCommand.execute();

    // Verify motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED / 2, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED / 2, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-DriveSubsystem.DRIVE_MAX_LINEAR_SPEED / 2, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+DriveSubsystem.DRIVE_MAX_LINEAR_SPEED / 2, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot knows when to stop anti-tip")
  public void isFinished() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged(); 
    inputs.rollAngle = +4.0;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Check command finished condition
    boolean value = m_antiTipCommand.isFinished();

    // Assert true
    assertEquals(true, value);
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot stops and locks modules when anti-tip is complete")
  public void end() {
    // Try to end command
    m_antiTipCommand.end(false);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lFrontDriveMotor, times(1)).stopMotor();
    verify(m_lFrontRotateMotor, times(1)).stopMotor();
    verify(m_rFrontDriveMotor, times(1)).stopMotor();
    verify(m_rFrontRotateMotor, times(1)).stopMotor();
    verify(m_lRearDriveMotor, times(1)).stopMotor();
    verify(m_lRearRotateMotor, times(1)).stopMotor();
    verify(m_lRearRotateMotor, times(1)).stopMotor();
    verify(m_rRearDriveMotor, times(1)).stopMotor();
    verify(m_rRearRotateMotor, times(1)).stopMotor();
  }
}
