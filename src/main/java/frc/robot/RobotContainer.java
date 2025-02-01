// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.WaggleSubsystem;
import frc.robot.subsystems.drive.CTREDriveSubsystem;
import frc.robot.subsystems.drive.REVDriveSubsystem;

public class RobotContainer {
  private static final CTREDriveSubsystem CTRE_DRIVE_SUBSYSTEM = new CTREDriveSubsystem(
    CTREDriveSubsystem.initializeHardware(),
    Constants.Drive.DRIVE_ROTATE_PID,
    Constants.Drive.DRIVE_AUTO_AIM_PID, Constants.Drive.DRIVE_CONTROL_CENTRICITY,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD
  );

  private static final REVDriveSubsystem REV_DRIVE_SUBSYSTEM = new REVDriveSubsystem(
    REVDriveSubsystem.initializeHardware(),
    Constants.Drive.DRIVE_ROTATE_PID,
    Constants.Drive.DRIVE_AUTO_AIM_PID, Constants.Drive.DRIVE_CONTROL_CENTRICITY,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD
  );

  private static final WaggleSubsystem WIGGLE_STICK = new WaggleSubsystem(Constants.WiggleStick.WIGGLE_STICK_CONFIG, Constants.WiggleStick.CONSTRAINTS);

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(Constants.HID.PRIMARY_CONTROLLER_PORT);

  private static SendableChooser<Command> m_automodeChooser = new SendableChooser<>();

  public RobotContainer() {
    // Set drive command
    CTRE_DRIVE_SUBSYSTEM.setDefaultCommand(
      CTRE_DRIVE_SUBSYSTEM.driveCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX()
      )
    );

    // Setup AutoBuilder
    CTRE_DRIVE_SUBSYSTEM.configureAutoBuilder();

    autoModeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);

    // Bind buttons and triggers
    configureBindings();
  }

  private void configureBindings() {
    // Start button - toggle traction control
    PRIMARY_CONTROLLER.start().onTrue(CTRE_DRIVE_SUBSYSTEM.toggleTractionControlCommand());

    PRIMARY_CONTROLLER.povLeft().onTrue(CTRE_DRIVE_SUBSYSTEM.resetPoseCommand(() -> new Pose2d()));

    // Left/right bumper - wiggle stick
    PRIMARY_CONTROLLER.leftBumper().onTrue(WIGGLE_STICK.setPositionCommand(0.0));
    PRIMARY_CONTROLLER.rightBumper().onTrue(WIGGLE_STICK.setPositionCommand(15.0));
  }

    /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", Commands.none());
  }

  /**
   * Get currently selected autonomous command
   * @return Autonomous command
   */
  public Command getAutonomousCommand() {
    return m_automodeChooser.getSelected();
  }
}
