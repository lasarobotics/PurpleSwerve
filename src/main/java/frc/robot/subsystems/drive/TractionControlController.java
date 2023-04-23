package frc.robot.subsystems.drive;

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.MathUtil;

public class TractionControlController {
  private enum State {
    DISABLED, ENABLED;
  }

  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.2;

  private final double MIN_SLIP_RATIO = 0.01;
  private final double MAX_SLIP_RATIO = 0.15;

  private double m_optimalSlipRatio = 0.0;
  private double m_deadband = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private State m_state = State.ENABLED;

  private HashMap<Double, Double> m_throttleInputMap = new HashMap<Double, Double>();

  /**
   * Create an instance of TractionControlController
   * @param optimalSlipRatio Desired slip ratio [+0.01, +0.15]
   * @param maxLinearSpeed maximum linear speed of robot (m/s)
   * @param deadband Deadband for controller input [+0.001, +0.1]
   * @param tractionControlCurve Expression characterising traction of the robot with "X" as the variable
   * @param throttleInputCurve Expression characterising throttle input with "X" as the variable
   */
  public TractionControlController(double optimalSlipRatio, double maxLinearSpeed, double deadband, PolynomialSplineFunction throttleInputCurve) {
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed * 1000) / 1000;
    this.m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);

    // Fill throttle input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000;
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate value between [0.0, +MAX_LINEAR_SPEED]
      double value = MathUtil.clamp(throttleInputCurve.value(deadbandKey), 0.0, +maxLinearSpeed);
      // Add both positive and negative values to map
      m_throttleInputMap.put(+key, +value);
      m_throttleInputMap.put(-key, -value);
    }
  }

  /**
   * Returns the next output of the traction control controller
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param velocityRequest Speed request (m/s)
   * @param wheelSpeed Linear wheel speed (m/s)
   * @return Optimal motor speed output (m/s)
   */
  public double calculate(double velocityRequest, double inertialVelocity, double wheelSpeed) {
    double velocityOutput = velocityRequest;

    // Make sure wheel speed and inertial velocity are positive
    wheelSpeed = Math.abs(wheelSpeed);
    inertialVelocity = Math.abs(inertialVelocity);
    
    // Apply basic traction control
    // Check slip ratio
    double currentSlipRatio = ((wheelSpeed - inertialVelocity) / inertialVelocity) * m_state.ordinal();
    // Limit wheel speed if slipping excessively
    if (currentSlipRatio > m_optimalSlipRatio)
      velocityOutput = Math.copySign(m_optimalSlipRatio * inertialVelocity + inertialVelocity, velocityRequest);

    return MathUtil.clamp(velocityOutput, -m_maxLinearSpeed, +m_maxLinearSpeed);
  }

  /**
   * Lookup velocity given throttle input
   * @param throttleLookup Throttle input [-1.0, +1.0]
   * @return Corresponding velocity
   */
  public double throttleLookup(double throttleLookup) {
    throttleLookup = Math.copySign(Math.floor(Math.abs(throttleLookup) * 1000) / 1000, throttleLookup) + 0.0;
    throttleLookup = MathUtil.clamp(throttleLookup, -1.0, +1.0);
    
    return m_throttleInputMap.get(throttleLookup);
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_state = (m_state.equals(State.ENABLED)) ? State.DISABLED : State.ENABLED;
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_state = State.ENABLED;
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_state = State.DISABLED;
  }

  /**
   * Is traction control enabled
   * @return true if enabled
   */
  public boolean isEnabled() {
    return m_state.equals(State.ENABLED);
  }
}
