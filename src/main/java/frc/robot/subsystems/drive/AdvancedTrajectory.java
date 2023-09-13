// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class AdvancedTrajectory {
  private final PIDController XY_PID_CONTROLLER = new PIDController(1.0, 0.0, 0.0, Constants.Global.ROBOT_LOOP_PERIOD);
  private final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(5.0, 0.0, 0.8, 
    new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI / 2), Constants.Global.ROBOT_LOOP_PERIOD);

  DriveSubsystem m_driveSubsystem;
  SwerveControllerCommand m_swerveCommand;
  Trajectory m_trajectory;

  /**
   * Creates new path trajectory using a physical x,y coordinate points
   * @param driveSubsystem DriveSubsystem required for drivetrain movement
   * @param waypoints List of x, y coordinate pairs in trajectory
   * @param goal Destination goal pose
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AdvancedTrajectory(DriveSubsystem driveSubsystem, List<Translation2d> waypoints, Pose2d goal, double maxVelocity, double maxAcceleration) {
    this.m_driveSubsystem = driveSubsystem;

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration).setKinematics(driveSubsystem.getKinematics());
    THETA_PID_CONTROLLER.enableContinuousInput(-Math.PI, +Math.PI);

    // Generate path from waypoints
    m_trajectory = TrajectoryGenerator.generateTrajectory(
      driveSubsystem.getPose(),
      waypoints,
      goal,
      trajectoryConfig
    );

    // Set swerve command
    m_swerveCommand = new SwerveControllerCommand(
      m_trajectory,
      driveSubsystem::getPose,
      driveSubsystem.getKinematics(),
      XY_PID_CONTROLLER,
      XY_PID_CONTROLLER,
      THETA_PID_CONTROLLER,
      driveSubsystem::autoDrive,
      driveSubsystem
    );
  }

  /**
   * Reset drive odometry to beginning of this path
   */
  private void resetOdometry() {
    m_driveSubsystem.resetPose(m_trajectory.getInitialPose());
  }

  /**
   * Get Ramsete command to run
   * @return Ramsete command that will stop when complete
   */
  public SequentialCommandGroup getCommandAndStop() {
    return m_swerveCommand.andThen(() -> {
            m_driveSubsystem.resetTurnPID();
            m_driveSubsystem.lock();
            m_driveSubsystem.stop();
           });
  }

  /**
   * Get auto command to execute path, resetting odometry first
   * @param isFirstPath true if path is the first one in autonomous
   * @return Ramsete command that will stop when complete
   */
  public SequentialCommandGroup getCommandAndStop(boolean isFirstPath) {
    if (isFirstPath) {
      return new InstantCommand(() -> resetOdometry())
                 .andThen(m_swerveCommand)
                 .andThen(() -> {
                    m_driveSubsystem.resetTurnPID();
                    m_driveSubsystem.lock();
                    m_driveSubsystem.stop();
                  });
    } else return getCommandAndStop();
  }
  
  /**
   * Get auto command to execute path
   * @return Ramsete command that does NOT stop when complete
   */
  public SequentialCommandGroup getCommand() {
    return m_swerveCommand.andThen(() -> m_driveSubsystem.resetTurnPID());
  }

  /**
   * Get auto command to execute path, resetting odometry first
   * @param isFirstPath true if path is first one in autonomous
   * @return Ramsete command that does NOT stop when complete
   */
  public SequentialCommandGroup getCommand(boolean isFirstPath) {
    if (isFirstPath) {
      return new InstantCommand(() -> resetOdometry())
                 .andThen(m_swerveCommand)
                 .andThen(() -> m_driveSubsystem.resetTurnPID());
    } else return getCommand();  
  }
}