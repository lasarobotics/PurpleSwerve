// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
import java.util.function.Supplier;

import org.lasarobotics.utils.JSONObject;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** PurplePath Client */
public class PurplePathClient {
  private final String GOAL_POSE_LOG_ENTRY = "/GoalPose";
  private final String FINAL_APPROACH_POSE_LOG_ENTRY = "/FinalApproachPose";

  private final String URI;

  private Supplier<Pose2d> m_poseSupplier;
  private PathConstraints m_pathConstraints;
  private HttpURLConnection m_serverConnection;
  private boolean m_isConnected;

  public PurplePathClient(Supplier<Pose2d> poseSupplier, PathConstraints pathConstraints) {
    this.m_poseSupplier = poseSupplier;
    this.m_pathConstraints = pathConstraints;
    this.m_isConnected = false;

    // Set URI
    if (RobotBase.isSimulation()) URI = "http://localhost:5000/";
    else URI = "http://purplebox.local:5000/";

    // Initialize connection
    getCommand(new Pose2d(), new PurplePathPose(new Pose2d(), 0.0));
  }

  /**
   * Get trajectory command to go from start pose to goal pose
   * @param start Starting pose of robot
   * @param goal Desired pose of robot
   * @return Command that drives robot to desired pose
   */
  private Command getCommand(Pose2d start, PurplePathPose goal) {
    Pose2d goalPose = goal.getGoalPose();
    Pose2d finalApproachPose = goal.getFinalApproachPose();
    PathPlannerPath finalApproachPath = goal.getFinalApproachPath();
    double finalApproachDistance = goal.getM_finalApproachDistance();

    if (goalPose == null || finalApproachPose == null || finalApproachPath == null) return Commands.none();

    // Check if robot is close to goal
    boolean isClose = start.getTranslation().getDistance(goalPose.getTranslation()) < finalApproachDistance;

    // Construct JSON request
    String jsonRequest = isClose ? JSONObject.writePointList(Arrays.asList(start.getTranslation(), goalPose.getTranslation()))
                                 : JSONObject.writePointList(Arrays.asList(start.getTranslation(), finalApproachPose.getTranslation()));

    // Send pathfinding request and get response
    String jsonResponse = "";
    try {
      // Define the server endpoint to send the HTTP request to
      m_serverConnection = (HttpURLConnection)new URL(URI).openConnection();

      // Indicate that we want to write to the HTTP request body
      m_serverConnection.setDoOutput(true);
      m_serverConnection.setRequestMethod("POST");
      m_serverConnection.setRequestProperty("Content-Type", "application/json");

      // Writing the post data to the HTTP request body
      BufferedWriter httpRequestBodyWriter = new BufferedWriter(new OutputStreamWriter(m_serverConnection.getOutputStream()));
      httpRequestBodyWriter.write(jsonRequest);
      httpRequestBodyWriter.close();

      // Reading from the HTTP response body
      Scanner httpResponseScanner = new Scanner(m_serverConnection.getInputStream());
      while (httpResponseScanner.hasNextLine()) jsonResponse += httpResponseScanner.nextLine();
      m_isConnected = m_serverConnection.getResponseCode() == 200;
      httpResponseScanner.close();
    } catch (IOException e) {
      System.out.println(e.getMessage());
      m_isConnected = false;
      return Commands.none();
    }

    // Attempt to read path from response
    List<Translation2d> points = JSONObject.readPointList(jsonResponse);

    // If path isn't there, return empty command
    if (points == null || points.size() < 2) return Commands.none();

    // Convert to PathPoint list
    List<PathPoint> waypoints = new ArrayList<>();
    for (var point : points)
      waypoints.add(new PathPoint(point, goalPose.getRotation()));

    // Generate path
    PathPlannerPath path = PathPlannerPath.fromPathPoints(
      waypoints,
      m_pathConstraints,
      new GoalEndState(
        isClose ? 0.0 : Math.sqrt(2 * m_pathConstraints.getMaxAccelerationMpsSq() * finalApproachDistance) * 0.66,
        finalApproachPose.getRotation()
      )
    );

    Logger.recordOutput(getClass().getSimpleName() + GOAL_POSE_LOG_ENTRY, goalPose);
    Logger.recordOutput(getClass().getSimpleName() + FINAL_APPROACH_POSE_LOG_ENTRY, finalApproachPose);

    // Return path following command
    return isClose ? AutoBuilder.followPathWithEvents(path)
                   : Commands.sequence(
                      AutoBuilder.followPathWithEvents(path),
                      AutoBuilder.followPathWithEvents(finalApproachPath)
                    );
  }

  /**
   * Get command to execute trajectory
   * @param goal Goal pose
   * @return Trajectory command
   */
  public Command getTrajectoryCommand(PurplePathPose goal) {
    return getCommand(m_poseSupplier.get(), goal);
  }

  /**
   * Get if connected to coprocessor
   * @return True if connected to PurplePath server on coprocessor
   */
  public boolean isConnected() {
    return m_isConnected;
  }
}
