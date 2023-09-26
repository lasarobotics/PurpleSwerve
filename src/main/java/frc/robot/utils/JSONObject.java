// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class JSONObject {
  private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();
  private static class JSONPose {
    @JsonProperty("x") double x;
    @JsonProperty("y") double y;
    @JsonProperty("rotation") double rotation;
    @JsonProperty("timestamp") double timestamp;

    private Pose2d toJavaType() {
      return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    }
  }

  private static class JSONPoint {
    @JsonProperty("x") double x;
    @JsonProperty("y") double y;

    private Translation2d toJavaType() {
      return new Translation2d(x, y);
    }
  }

  private static class JSONPoseList {
    @JsonProperty("poses") List<JSONPose> poses;

    private List<Pose2d> toJavaType() {
      List<Pose2d> poseList = new ArrayList<Pose2d>();
      for (var pose : poses) poseList.add(pose.toJavaType());

      return poseList;
    }
  }

  private static class JSONPointList {
    @JsonProperty("path") List<JSONPoint> points;

    private List<Translation2d> toJavaType() {
      List<Translation2d> pointList = new ArrayList<Translation2d>();
      for (var point : points) pointList.add(point.toJavaType());

      return pointList;
    }
  }

  public static Pose2d getPose(String json) {
    try { return OBJECT_MAPPER.readValue(json, JSONPose.class).toJavaType(); } 
    catch (Exception e) {
      System.out.println(e.getMessage());
      return null;
    }
  }

  public static Translation2d getPoint(String json) {
    try { return OBJECT_MAPPER.readValue(json, JSONPoint.class).toJavaType(); } 
    catch (Exception e) {
      System.out.println(e.getMessage());
      return null;
    }
  }

  public static List<Pose2d> getPoseList(String json) {
    try { return OBJECT_MAPPER.readValue(json, JSONPoseList.class).toJavaType(); }
    catch (Exception e) {
      System.out.println(e.getMessage());
      return null;
    }
  }

  public static List<Translation2d> getPointList(String json) {
    try { return OBJECT_MAPPER.readValue(json, JSONPointList.class).toJavaType(); } 
    catch (Exception e) {
      System.out.println(e.getMessage());
      return null;
    }
  }
}
