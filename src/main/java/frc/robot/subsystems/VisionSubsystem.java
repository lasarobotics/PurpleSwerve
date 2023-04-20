// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.utils.VisionCamera;

public class VisionSubsystem {
  public static class Hardware {
    private VisionCamera[] cameras;

    public Hardware(VisionCamera... cameras) {
      this.cameras = cameras;
    }
  }

  private static VisionSubsystem m_subsystem;

  private AprilTagFieldLayout m_fieldLayout;

  private VisionCamera[] m_cameras;
  private PhotonPoseEstimator[] m_poseEstimators;

  /**
   * Create a new vision subsystem
   * @param visionHardware Vision hardware
   */
  private VisionSubsystem(Hardware visionHardware) {
    this.m_cameras = visionHardware.cameras;

    // Attempt to load AprilTag field layout
    try {
      m_fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) { e.printStackTrace(); }

    // Initialize pose estimators for all cameras
    for (int i = 0; i < m_cameras.length; i++) {
      m_poseEstimators[i] = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP, m_cameras[i].getCamera(), m_cameras[i].getTransform());
      m_poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // Set all cameras to primary pipeline
    for (VisionCamera camera : m_cameras) 
      camera.getCamera().setPipelineIndex(0);
  }

  public static VisionSubsystem getInstance() {
    if (m_subsystem == null) m_subsystem = new VisionSubsystem(initializeHardware());
    return m_subsystem;
  }

  /**
   * Get AprilTag field layout
   * @return field layout
   */
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return this.m_fieldLayout;
  }

  public static Hardware initializeHardware() {
    Hardware visionHardware = new Hardware(new VisionCamera(Constants.VisionHardware.CAMERA_0_NAME, Constants.VisionHardware.CAMERA_0_LOCATION));

    return visionHardware;
  }

  /**
   * Get currently estimated robot pose
   * @param prevEstimatedRobotPose The current best guess at robot pose
   * @return  an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create the estimate
   */
  public EstimatedRobotPose[] getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    EstimatedRobotPose estimatedPoses[] = new EstimatedRobotPose[m_poseEstimators.length];

    for (int i = 0; i < m_poseEstimators.length; i++) {
      Optional<EstimatedRobotPose> result;
      m_poseEstimators[i].setReferencePose(prevEstimatedRobotPose);
      result = m_poseEstimators[i].update();
      if (result.isPresent()) estimatedPoses[i] = result.get();
    }
   
    return estimatedPoses;
  }
}
