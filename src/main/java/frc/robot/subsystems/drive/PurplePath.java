// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReferenceArray;

import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Notifier;

/** PurplePath Pathfinder */
public class PurplePath {
  public static final int MAX_TRAJECTORIES = 4;
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
  private final PubSubOption[] PUBSUB_OPTIONS = { PubSubOption.periodic(GlobalConstants.ROBOT_LOOP_PERIOD), PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10) };
  private StringSubscriber[] m_trajectorySubscribers =  new StringSubscriber[MAX_TRAJECTORIES];
  private AtomicReferenceArray<Trajectory> m_trajectories;
  private List<Pose2d> m_goals;
  private StringPublisher m_posePublisher;
  private StringPublisher m_goalPublisher;
  private Notifier m_thread;

  public PurplePath() {
    // Initialise subcribers
    NetworkTable table = NetworkTableInstance.getDefault().getTable(NETWORK_TABLE_NAME);
    for (int i = 0; i < MAX_TRAJECTORIES; i++)
      m_trajectorySubscribers[i] = table.getStringTopic(TRAJECTORY_LOG_ENTRY[i])
        .subscribe("", PUBSUB_OPTIONS);

    // Initialise trajectories
    m_trajectories = new AtomicReferenceArray<>(MAX_TRAJECTORIES);

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
      List<Translation2d> path = JSONObject.readPointList(m_trajectorySubscribers[i].getAtomic().value);
      // If path isn't there, clear trajectory
      if (path == null) {
        m_trajectories.set(i, new Trajectory());
        continue;
      }

      // If path exists, generate trajectory
      List<Trajectory.State> trajectory = new ArrayList<>();
      for (var point : path)
        trajectory.add(new Trajectory.State(0.0, 0.0, 0.0, new Pose2d(point, Rotation2d.fromDegrees(0.0)), 0.0));

      // Update trajectory
      m_trajectories.set(i, new Trajectory(trajectory));
    }
  }

  /**
   * Start runner thread
   */
  public void startThread() {
    m_thread = new Notifier(() -> run());
    m_thread.setName("PurplePath");
    m_thread.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD);
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
   * Get latest calculated trajectory
   * @param index Index of trajectory to obtain
   * @return Calculated trajectory
   */
  public Trajectory getLatestTrajectory(int index) {
    if (index < 0 || index > MAX_TRAJECTORIES) return new Trajectory();
    return  m_trajectories.get(index);
  }
}
