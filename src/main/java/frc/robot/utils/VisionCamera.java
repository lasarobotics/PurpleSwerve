// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

/** Create a camera */
 public class VisionCamera {
  private PhotonCamera m_camera;
  private Transform3d m_transform;

  public VisionCamera(String name, Transform3d location) {
    this.m_camera = new org.photonvision.PhotonCamera(name);
    this.m_transform = location;
  }

  public PhotonCamera getCamera() { return m_camera; }

  public Transform3d getTransform() { return m_transform; }
}
