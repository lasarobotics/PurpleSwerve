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
import frc.robot.subsystems.drive.AutoTrajectory;
import frc.robot.subsystems.drive.CTREDriveSubsystem;
import frc.robot.subsystems.drive.REVDriveSubsystem;

public class RobotContainer {
  private static final CTREDriveSubsystem CTREDRIVE_SUBSYSTEM = new CTREDriveSubsystem(
    CTREDriveSubsystem.initializeHardware(),
    Constants.Drive.DRIVE_ROTATE_PID,
    Constants.Drive.DRIVE_AUTO_AIM_PID, Constants.Drive.DRIVE_CONTROL_CENTRICITY,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD
  );

  private static final REVDriveSubsystem REVDRIVE_SUBSYSTEM = new REVDriveSubsystem(
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
    CTREDRIVE_SUBSYSTEM.setDefaultCommand(
      CTREDRIVE_SUBSYSTEM.driveCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX()
      )
    );

    // Setup AutoBuilder
    CTREDRIVE_SUBSYSTEM.configureAutoBuilder();

    autoModeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);

    // Bind buttons and triggers
    configureBindings();
  }

  private void configureBindings() {
    // Start button - toggle traction control
    PRIMARY_CONTROLLER.start().onTrue(CTREDRIVE_SUBSYSTEM.toggleTractionControlCommand());

    // A button - go to amp
    PRIMARY_CONTROLLER.a().whileTrue(
      CTREDRIVE_SUBSYSTEM.goToPoseCommand(
        Constants.Field.AMP
      )
    );

    // B button - go to source
    PRIMARY_CONTROLLER.b().whileTrue(CTREDRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.SOURCE));

    PRIMARY_CONTROLLER.povLeft().onTrue(CTREDRIVE_SUBSYSTEM.resetPoseCommand(() -> new Pose2d()));

    // Left/right bumper - wiggle stick
    PRIMARY_CONTROLLER.leftBumper().onTrue(WIGGLE_STICK.setPositionCommand(0.0));
    PRIMARY_CONTROLLER.rightBumper().onTrue(WIGGLE_STICK.setPositionCommand(15.0));
  }

    /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", Commands.none());
    m_automodeChooser.addOption("Leave", new AutoTrajectory(CTREDRIVE_SUBSYSTEM, "Leave").getCommand());
    m_automodeChooser.addOption("Preload + 3 Ring", new AutoTrajectory(CTREDRIVE_SUBSYSTEM, "Preload + 3 Ring").getCommand());
    m_automodeChooser.addOption("Preload + 1", new AutoTrajectory(CTREDRIVE_SUBSYSTEM, "Preload + 1").getCommand());
  }

  /**
   * Get currently selected autonomous command
   * @return Autonomous command
   */
  public Command getAutonomousCommand() {
    return m_automodeChooser.getSelected();
  }
}
