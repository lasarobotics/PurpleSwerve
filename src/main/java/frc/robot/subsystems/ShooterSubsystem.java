// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.lasarobotics.utils.AimAssist;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static final String MECHANISM_2D_LOG_ENTRY = "/Mechanism2d";
  private LoggedMechanism2d m_mechanism2d;
  private LoggedMechanismLigament2d m_simShooterJoint;
  private Supplier<Distance> m_targetDistanceSupplier;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Supplier<Distance> targetDistanceSupplier) {
    m_targetDistanceSupplier = targetDistanceSupplier;
    m_mechanism2d = new LoggedMechanism2d(1.0, 1.0);
    m_simShooterJoint = m_mechanism2d.getRoot("shooter", 0.5, 0.33).append(new LoggedMechanismLigament2d("shooter", 0.4, 1.0));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    Pair<Angle, LinearVelocity> solutions = AimAssist.getShooterValues(m_targetDistanceSupplier.get(), Constants.Field.HUB_HEIGHT.minus(Constants.Shooter.SHOOTER_HEIGHT), Units.Degrees.of(-60));
    m_simShooterJoint.setAngle(solutions.getFirst());

    Logger.recordOutput(getName() + "/Angle", solutions.getFirst());
    Logger.recordOutput(getName() + "/Velocity", solutions.getSecond());
    Logger.recordOutput(getName() + MECHANISM_2D_LOG_ENTRY, m_mechanism2d);
  }
}
