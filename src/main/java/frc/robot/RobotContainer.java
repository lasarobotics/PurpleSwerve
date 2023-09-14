// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.ZoneId;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  public static final boolean REAL_HARDWARE = true;
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
    DriveSubsystem.initializeHardware(REAL_HARDWARE),
    Constants.Drive.DRIVE_TURN_PID.kP,
    Constants.Drive.DRIVE_TURN_PID.kD,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD,
    Constants.Drive.DRIVE_SLIP_RATIO,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE
  );

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(Constants.HID.PRIMARY_CONTROLLER_PORT);

  private static boolean m_isTimeSynced = false;

  public RobotContainer() {
    // Make sure time is synced to DS
    syncTime();

    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
      new RunCommand(
        () -> DRIVE_SUBSYSTEM.teleopPID(PRIMARY_CONTROLLER.getLeftY(), PRIMARY_CONTROLLER.getLeftX(), PRIMARY_CONTROLLER.getRightX()), 
        DRIVE_SUBSYSTEM
      )
    );

    // Bind buttons and triggers
    configureBindings();
  }

  private void configureBindings() {
    PRIMARY_CONTROLLER.start().onTrue(new InstantCommand(() -> DRIVE_SUBSYSTEM.toggleTractionControl(), DRIVE_SUBSYSTEM));
  }

  /**
   * Wait until RoboRIO syncs time with DS
   */
  private void syncTime() {
    int WAIT_TIME = (int)(6 / Constants.Global.ROBOT_LOOP_PERIOD);
    Timer dsConnectedTimer = new Timer();
    ZoneId UTC = ZoneId.of("UTC");

    dsConnectedTimer.start();

    while (!m_isTimeSynced) {
      if (DriverStation.isDSAttached()) {
        System.out.println("Waiting to sync time with DS...");
        dsConnectedTimer.get();
      } else {
        System.out.println("DS not connected...");
        dsConnectedTimer.restart();
      }

      // If connected to DS for more than 6 seconds
      if (dsConnectedTimer.get() > WAIT_TIME) {
        // Check if time is synced
        m_isTimeSynced = LocalDateTime.now(UTC).getYear() > 2000;

        // If time not synced, wait a bit longer...
        if (!m_isTimeSynced) dsConnectedTimer.restart();
      }
    }
    System.out.println("Time synced!");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
