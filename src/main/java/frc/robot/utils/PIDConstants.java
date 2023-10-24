// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utils;

public class PIDConstants {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kF;
  public final double period;

  public PIDConstants(double kP, double kI, double kD, double kF, double period) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.period = period;
  }

  public PIDConstants(double kP, double kI, double kD, double kF) {
    this(kP, kI, kD, kF, 0.02);
  }
}