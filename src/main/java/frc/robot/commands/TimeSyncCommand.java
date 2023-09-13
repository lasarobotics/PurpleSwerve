// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.LocalDateTime;
import java.time.ZoneId;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TimeSyncCommand extends CommandBase {
  private static final ZoneId UTC = ZoneId.of("UTC");
  private static final int WAIT_TIME = (int)(6 / Constants.Global.ROBOT_LOOP_PERIOD);
  private boolean m_isTimeSynced;
  private int m_dsAttachCount;
  
  /** Creates a new TimeSyncCommand. */
  public TimeSyncCommand() {
    m_isTimeSynced = false;
    m_dsAttachCount = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isDSAttached()) m_dsAttachCount++;
    else m_dsAttachCount = 0;

    // If connected to DS for more than 6 seconds
    if (m_dsAttachCount > WAIT_TIME) {
      // Check if time is synced
      m_isTimeSynced = LocalDateTime.now(UTC).getYear() > 2000;

      // If time not synced, wait a bit longer...
      if (!m_isTimeSynced) m_dsAttachCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isTimeSynced;
  }
}
