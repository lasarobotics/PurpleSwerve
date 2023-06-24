package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;

public class TractionControlController {
  private enum State {
    DISABLED {
      @Override
      public State toggle() { return ENABLED; }
    },
    ENABLED {
      @Override
      public State toggle() { return DISABLED; }
    };

    public abstract State toggle(); 
  }

  private final double MIN_SLIP_RATIO = 0.01;
  private final double MAX_SLIP_RATIO = 0.40;
  private final int FILTER_WINDOW_SIZE = 5;

  private double m_averageWheelSpeed = 0.0;
  private double m_optimalSlipRatio = 0.0;
  private double m_currentSlipRatio = 0.0;
  private double m_maxLinearSpeed = 0.0;
  private boolean m_isSlipping = false;
  private State m_state = State.ENABLED;

  private LinearFilter m_speedFilter;

  /**
   * Create an instance of TractionControlController
   * @param optimalSlipRatio Desired slip ratio [+0.01, +0.40]
   * @param maxLinearSpeed maximum linear speed of robot (m/s)
   */
  public TractionControlController(double optimalSlipRatio, double maxLinearSpeed) {
    this.m_optimalSlipRatio = MathUtil.clamp(optimalSlipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
    this.m_maxLinearSpeed = Math.floor(maxLinearSpeed * 1000) / 1000;
    this.m_speedFilter = LinearFilter.movingAverage(FILTER_WINDOW_SIZE); 
  }

  private void updateSlipRatio(double wheelSpeed, double inertialVelocity) {
    // Calculate average speed using moving average filter
    m_averageWheelSpeed = m_speedFilter.calculate(wheelSpeed);

    // Calculate current slip ratio
    m_currentSlipRatio = ((wheelSpeed - m_averageWheelSpeed) / inertialVelocity) * m_state.ordinal();

    // Check if wheel is slipping
    m_isSlipping = m_currentSlipRatio > m_optimalSlipRatio;
  }

  /**
   * Returns the next output of the traction control controller
   * @param velocityRequest Velocity request (m/s)
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param wheelSpeed Linear wheel speed (m/s)
   * @return Optimal motor speed output (m/s)
   */
  public double calculate(double velocityRequest, double inertialVelocity, double wheelSpeed) {
    double velocityOutput = velocityRequest;

    // Make sure wheel speed and inertial velocity are positive
    wheelSpeed = Math.abs(wheelSpeed);
    inertialVelocity = Math.abs(inertialVelocity);
    
    // Apply basic traction control
    // Limit wheel speed if slipping excessively
    updateSlipRatio(wheelSpeed, inertialVelocity);
    if (m_isSlipping) 
      velocityOutput = Math.copySign(m_optimalSlipRatio * inertialVelocity + m_averageWheelSpeed, velocityRequest);

    // Return corrected velocity output, clamping to max linear speed
    return MathUtil.clamp(velocityOutput, -m_maxLinearSpeed, +m_maxLinearSpeed);
  }

  /**
   * Is wheel slipping
   * @return True if wheel is slipping
   */
  public boolean isSlipping() {
    return m_isSlipping;
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_state = m_state.toggle();
    m_speedFilter.reset();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_state = State.ENABLED;
    m_speedFilter.reset();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_state = State.DISABLED;
    m_speedFilter.reset();
  }

  /**
   * Is traction control enabled
   * @return True if enabled
   */
  public boolean isEnabled() {
    return m_state.equals(State.ENABLED);
  }
}
