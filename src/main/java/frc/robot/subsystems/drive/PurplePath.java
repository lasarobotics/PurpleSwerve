// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReferenceArray;

import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.JSONObject;

import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** PurplePath Pathfinder */
public class PurplePath {
  public static final int MAX_TRAJECTORIES = 10;
  public static final String[] TRAJECTORY_LOG_ENTRY = {
    "Trajectory0",
    "Trajectory1",
    "Trajectory2",
    "Trajectory3",
    "Trajectory4",
    "Trajectory5",
    "Trajectory6",
    "Trajectory7",
    "Trajectory8",
    "Trajectory9",
  };

  private final String GOAL_LOG_ENTRY = "Goal";
  private final String POSE_LOG_ENTRY = "Pose";
  private final String NETWORK_TABLE_NAME = "PurplePath";
  private final PubSubOption[] PUBSUB_OPTIONS = { PubSubOption.periodic(GlobalConstants.ROBOT_LOOP_PERIOD / 2), PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10) };
  private DriveSubsystem m_driveSubsystem;
  private StringSubscriber[] m_trajectorySubscribers =  new StringSubscriber[MAX_TRAJECTORIES];
  private AtomicReferenceArray<Command> m_commands;
  private List<Pose2d> m_goals;
  private StringPublisher m_posePublisher;
  private StringPublisher m_goalPublisher;
  private Notifier m_thread;

  public PurplePath(DriveSubsystem driveSubsystem) {
    this.m_driveSubsystem = driveSubsystem;
    // Initialise subcribers
    NetworkTable table = NetworkTableInstance.getDefault().getTable(NETWORK_TABLE_NAME);
    for (int i = 0; i < MAX_TRAJECTORIES; i++)
      m_trajectorySubscribers[i] = table.getStringTopic(TRAJECTORY_LOG_ENTRY[i])
        .subscribe("", PUBSUB_OPTIONS);

    // Initialise commands
    m_commands = new AtomicReferenceArray<>(MAX_TRAJECTORIES);

    // Initialise goal publisher
    m_posePublisher = table.getStringTopic(POSE_LOG_ENTRY).publish(PUBSUB_OPTIONS);
    m_goalPublisher = table.getStringTopic(GOAL_LOG_ENTRY).publish(PUBSUB_OPTIONS);
  }

  /**
   * Get latest paths from PurplePath and calculate trajectories
   */
  private void run() {
    for (int i = 0; i < m_trajectorySubscribers.length; i++) {
      // Attempt to read path from NetworkTables
      List<Translation2d> points = JSONObject.readPointList(m_trajectorySubscribers[i].getAtomic().value);
      // If path isn't there, clear trajectory
      if (points == null) {
        m_commands.set(i, Commands.none());
        continue;
      }

      // Convert to PathPoint list
      List<PathPoint> waypoints = new ArrayList<>();
      for (var point : points)
        waypoints.add(new PathPoint(point, m_goals.get(i).getRotation()));

      // Update trajectory
      m_commands.set(i, new AutoTrajectory(m_driveSubsystem, waypoints, m_driveSubsystem.getPathConstraints()).getCommandAndStop());
    }
  }

  /**
   * Start runner thread
   */
  public void startThread() {
    m_thread = new Notifier(() -> run());
    m_thread.setName("PurplePath");
    m_thread.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD / 2);
  }

  /**
   * Set current robot pose
   * @param pose Current robot pose
   */
  public void setCurrentPose(Pose2d pose) {
    m_posePublisher.set(JSONObject.writePose(pose));
  }

  /**
   * Set goal poses
   * @param goals Desired goal poses
   */
  public void setGoals(List<Pose2d> goals) {
    m_goals = goals;
    m_goalPublisher.set(JSONObject.writePoseList(m_goals));
  }

  /**
   * Get command to execute trajectory
   * @param index Index of trajectory command to obtain
   * @return Trajectory command
   */
  public Command getTrajectoryCommand(int index) {
    if (index < 0 || index > MAX_TRAJECTORIES) return Commands.none();
    return  m_commands.get(index);
  }
}
