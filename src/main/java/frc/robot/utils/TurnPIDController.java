package frc.robot.utils;

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class TurnPIDController extends PIDController {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.2;
  private final double FILTER_FACTOR = 1.0 / 3.0;

  private HashMap<Double, Double> m_turnInputMap = new HashMap<Double, Double>();
  private double m_turnScalar = 0.0;
  private double m_lookAhead = 0.0;
  private double m_deadband = 0.0;
  private double m_turnRequest = 0.0;
  private boolean m_isTurning = false;

  /**
   * Create an instance of TurnPIDController
   * @param kP The proportional coefficient
   * @param kD The derivative coefficient
   * @param turnScalar Value to turn input by (degrees)
   * @param deadband Controller deadband
   * @param lookaAhead Number of loops to look ahead by
   * @param turnInputCurve Turn input curve
   */
  public TurnPIDController(double kP, double kD, double turnScalar, double deadband, double lookAhead, PolynomialSplineFunction turnInputCurve) {
    super(kP, 0.0, kD, Constants.Global.ROBOT_LOOP_PERIOD);
    this.m_turnScalar = turnScalar;
    this.m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);
    this.m_lookAhead = lookAhead;

    // Fill turn input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000; 
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate and clamp value between [0.0, +1.0]
      double value = MathUtil.clamp(turnInputCurve.value(deadbandKey), 0.0, +1.0);
      // Add both positive and negative values to map
      m_turnInputMap.put(+key, +value);
      m_turnInputMap.put(-key, -value);
    }
  }

  /**
   * Returns next output of TurnPIDController
   * @param currentAngle current yaw angle of robot (degrees)
   * @param turnRate current yaw turn rate of robot (degrees/sec)
   * @param turnRequest turn request [-1.0, +1.0]
   * 
   * @return optimal turn output [-1.0, +1.0]
   */
  public double calculate(double currentAngle, double turnRate, double turnRequest) {
    // Filter turnRequest
    m_turnRequest -= (m_turnRequest - turnRequest) * FILTER_FACTOR;

    // Start turning if input is greater than deadband
    if (Math.abs(m_turnRequest) >= m_deadband) {
      // Get scaled turnRequest
      m_turnRequest = Math.copySign(Math.floor(Math.abs(m_turnRequest) * 1000) / 1000, m_turnRequest) + 0.0;
      double scaledTurnRequest = m_turnInputMap.get(m_turnRequest);
      // Add delta to setpoint scaled by factor
      super.setSetpoint(currentAngle + (scaledTurnRequest * m_turnScalar));
      m_isTurning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (m_isTurning) {
        super.setSetpoint(currentAngle + (turnRate * m_lookAhead * Constants.Global.ROBOT_LOOP_PERIOD));
        m_isTurning = false;
      }
    }

    return MathUtil.clamp(super.calculate(currentAngle), -1.0, +1.0);
  }

  /**
   * Get if robot is turning
   * @return true if turning
   */
  public boolean isTurning() {
    return m_isTurning;
  }
}
