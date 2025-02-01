// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class WaggleSubsystem extends SubsystemBase {
  Spark m_motor;
  SparkBaseConfig m_config;
  Constraints m_constraint;

  /** Creates a new wiggleStick. */
  public WaggleSubsystem(SparkBaseConfig config, Constraints constraint) {
    m_motor = new Spark(new Spark.ID("wiggleStick", 20), MotorKind.NEO);
    m_config = config;
    m_constraint = constraint;
    m_config = (m_motor.getKind().equals(MotorKind.NEO_VORTEX)) ? new SparkFlexConfig() : new SparkMaxConfig();
    double conversionFactor = 1.0;
    m_config.encoder.positionConversionFactor(conversionFactor);
    m_config.encoder.velocityConversionFactor(conversionFactor / 60);
    m_config.closedLoop.pidf(0.2, 0.0, 0.0, 0);
    m_config.closedLoop.maxMotion.maxAcceleration(20);
    m_config.closedLoop.maxMotion.maxVelocity(10);

    m_config.closedLoop.positionWrappingEnabled(true);
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.resetEncoder();
  }

  /** Moves to the given position */
  private void setPosition(double position) {
    m_motor.set(position, ControlType.kMAXMotionPositionControl);
  }

  /** Gets the position using the encoders */
  public double getPosition() {
    return m_motor.getInputs().encoderPosition;
  }

  /** Command to set the position of the robot */
  public Command setPositionCommand(double position) {
    return runOnce(() -> setPosition(position));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}