// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Consumer;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.MAXSwerveModule.ModuleLocation;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    boolean isHardwareReal;
    MAXSwerveModule lFrontModule;
    MAXSwerveModule rFrontModule;
    MAXSwerveModule lRearModule;
    MAXSwerveModule rRearModule;
    AHRS navx;

    public Hardware(boolean isHardwareReal,
                    MAXSwerveModule lFrontModule,
                    MAXSwerveModule rFrontModule,
                    MAXSwerveModule lRearModule,
                    MAXSwerveModule rRearModule,
                    AHRS navx) {
      this.isHardwareReal = isHardwareReal;
      this.lFrontModule = lFrontModule;
      this.rFrontModule = rFrontModule;
      this.lRearModule = lRearModule;
      this.rRearModule = rRearModule;
      this.navx = navx;
    }
  }

  // Drive specs, these numbers use the motor shaft encoder
  public static final double DRIVE_WHEELBASE = 0.6;
  public static final double DRIVE_TRACK_WIDTH = 0.6;
  public static final double DRIVE_WHEEL_DIAMETER_METERS = 0.0762; // 3" wheels
  public static final double DRIVE_GEAR_RATIO = 4.71;
  public static final double DRIVE_TICKS_PER_METER = (Constants.Global.NEO_ENCODER_TICKS_PER_ROTATION * DRIVE_GEAR_RATIO) * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI));
  public static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
  public static final double DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * Constants.Global.NEO_ENCODER_TICKS_PER_ROTATION;
  public static final double DRIVETRAIN_EFFICIENCY = 0.88;
  public static final double DRIVE_MAX_LINEAR_SPEED = (Constants.Global.NEO_MAX_RPM / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;
  public static final double DRIVE_AUTO_ACCELERATION = DRIVE_MAX_LINEAR_SPEED - 0.5;

  private TurnPIDController m_turnPIDController;
  private TractionControlController m_tractionControlController;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private MAXSwerveModule m_lFrontModule;
  private MAXSwerveModule m_rFrontModule;
  private MAXSwerveModule m_lRearModule;
  private MAXSwerveModule m_rRearModule;
  private AHRS m_navx;

  private final double TOLERANCE = 0.125;
  private final double TIP_THRESHOLD = 30.0;
  private final double BALANCED_THRESHOLD = 5.0;

  public final Command ANTI_TIP_COMMAND = new FunctionalCommand(
    () -> {},
    () -> { DriveSubsystem.this.antiTip(); },
    new Consumer<Boolean>() {
      public void accept(Boolean arg0) {
        DriveSubsystem.this.resetTurnPID();
        DriveSubsystem.this.lock();
        DriveSubsystem.this.stop();
      };
    },
    DriveSubsystem.this::isBalanced,
    this
  );

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param turnScalar Scalar for turn input (degrees)
   * @param deadband Deadband for controller input [+0.001, +0.2]
   * @param lookAhead Turn PID lookahead, in number of loops
   * @param slipRatio Tracion control slip ratio
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   * @param currentLimitConfiguration Drive current limit
   */
  public DriveSubsystem(Hardware drivetrainHardware, double kP, double kD,
                        double turnScalar, double deadband, double lookAhead, double slipRatio,
                        PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve) {
    m_turnPIDController = new TurnPIDController(kP, kD, turnScalar, deadband, lookAhead, turnInputCurve);
    m_tractionControlController =  new TractionControlController(slipRatio, DRIVE_MAX_LINEAR_SPEED, deadband, throttleInputCurve);

    this.m_lFrontModule = drivetrainHardware.lFrontModule;
    this.m_rFrontModule = drivetrainHardware.rFrontModule;
    this.m_lRearModule = drivetrainHardware.lRearModule;
    this.m_rRearModule = drivetrainHardware.rRearModule;
    this.m_navx = drivetrainHardware.navx;

    // Calibrate and reset navX
    m_navx.calibrate();
    while (m_navx.isCalibrating()) stop();
    m_navx.reset();

    // Setup turn PID
    m_turnPIDController.setTolerance(TOLERANCE);
    m_turnPIDController.setSetpoint(getAngle());

    // Define drivetrain kinematics
    m_kinematics = new SwerveDriveKinematics(m_lFrontModule.getModuleCoordinate(),
                                             m_rFrontModule.getModuleCoordinate(),
                                             m_lRearModule.getModuleCoordinate(),
                                             m_rRearModule.getModuleCoordinate());

    // Initialise pose estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
        m_lFrontModule.getPosition(),
        m_rFrontModule.getPosition(),
        m_lRearModule.getPosition(),
        m_rRearModule.getPosition()
      }, 
      new Pose2d()
    );

    // Setup anti-tip command
    new Trigger(this::isTipping).onTrue(ANTI_TIP_COMMAND);
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @param isHardwareReal True if hardware is real
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware(boolean isHardwareReal) {
    MAXSwerveModule lFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        isHardwareReal,
        Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.LeftFront, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    MAXSwerveModule rFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        isHardwareReal,
        Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.RightFront, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    MAXSwerveModule lRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        isHardwareReal,
        Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.LeftRear, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    MAXSwerveModule rRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        isHardwareReal,
        Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.RightRear, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte)(Constants.Global.ROBOT_LOOP_PERIOD * 2));

    Hardware drivetrainHardware = new Hardware(isHardwareReal, lFrontModule, rFrontModule, lRearModule, rRearModule, navx);

    return drivetrainHardware;  
  }

  /**
   * Set swerve modules
   * @param moduleStates Array of calculated module states
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates) {
    m_lFrontModule.set(moduleStates);
    m_rFrontModule.set(moduleStates);
    m_lRearModule.set(moduleStates);
    m_rRearModule.set(moduleStates);
  }

  /**
   * Drive robot and apply traction control
   * @param velocityX Desired X (forward) velocity in m/s
   * @param velocityY Desired Y (sideways) velocity in m/s
   * @param rotateRate Desired rotate rate in degrees per second
   */
  private void drive(double velocityX, double velocityY, double rotateRate) {
    // Convert speeds to module states
    SwerveModuleState[] moduleStates = 
      m_kinematics.toSwerveModuleStates(new ChassisSpeeds(velocityX, velocityY, Math.toRadians(rotateRate)));

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Apply traction control
    applyTractionControl(moduleStates);

    // Set modules to calculated states
    setSwerveModules(moduleStates);
  }

  /**
   * Apply traction control to all swerve modules
   * <p>
   * This method modifies the module states in place
   * @param moduleStates Calculated module states
   */
  private void applyTractionControl(SwerveModuleState[] moduleStates) {
    moduleStates[ModuleLocation.LeftFront.index].speedMetersPerSecond = m_tractionControlController.calculate(
      moduleStates[ModuleLocation.LeftFront.index].speedMetersPerSecond,
      m_lFrontModule.calculateRealSpeed(getInertialVelocity(), getTurnRate()),
      m_lFrontModule.getDriveVelocity()
    );
    moduleStates[ModuleLocation.RightFront.index].speedMetersPerSecond = m_tractionControlController.calculate(
      moduleStates[ModuleLocation.RightFront.index].speedMetersPerSecond,
      m_rFrontModule.calculateRealSpeed(getInertialVelocity(), getTurnRate()),
      m_rFrontModule.getDriveVelocity()
    );
    moduleStates[ModuleLocation.LeftRear.index].speedMetersPerSecond = m_tractionControlController.calculate(
      moduleStates[ModuleLocation.LeftRear.index].speedMetersPerSecond,
      m_lRearModule.calculateRealSpeed(getInertialVelocity(), getTurnRate()),
      m_lRearModule.getDriveVelocity()
    );
    moduleStates[ModuleLocation.RightRear.index].speedMetersPerSecond = m_tractionControlController.calculate(
      moduleStates[ModuleLocation.RightRear.index].speedMetersPerSecond,
      m_rRearModule.calculateRealSpeed(getInertialVelocity(), getTurnRate()),
      m_rRearModule.getDriveVelocity()
    );
  }

  /**
   * Reset drive encoders
   */
  private void resetEncoders() {
    m_lFrontModule.resetDriveEncoder();
    m_rFrontModule.resetDriveEncoder();
    m_lRearModule.resetDriveEncoder();
    m_rRearModule.resetDriveEncoder();
  }

  /**
   * Reset DriveSubsystem PID
   */
  public void resetTurnPID() {
    m_turnPIDController.setSetpoint(getAngle());
    m_turnPIDController.reset();
  }

  /**
   * Get current module positions
   * @return Array of swerve module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_lFrontModule.getPosition(),
      m_rFrontModule.getPosition(),
      m_lRearModule.getPosition(),
      m_rRearModule.getPosition()
    };
  }

  /**
   * Update robot pose
   */
  private void updatePose() {
    // Get estimated poses from VisionSubsystem
    List<EstimatedRobotPose> visionEstimatedRobotPoses = VisionSubsystem.getInstance().getEstimatedGlobalPose(getPose());

    // Update pose based on odometry
    m_poseEstimator.update(m_navx.getRotation2d(), getModulePositions());

    // Exit if no valid vision pose estimates
    if (visionEstimatedRobotPoses.isEmpty()) return;

    // Add vision measurements to pose estimator
    for (EstimatedRobotPose visionEstimatedRobotPose : visionEstimatedRobotPoses)
      m_poseEstimator.addVisionMeasurement(visionEstimatedRobotPose.estimatedPose.toPose2d(), visionEstimatedRobotPose.timestampSeconds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    smartDashboard();
  }

  /**
   * SmartDashboard indicators
   */
  public void smartDashboard() {
    SmartDashboard.putBoolean("TC", m_tractionControlController.isEnabled());
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed [-1.0, +1.0]
   */
  public void teleopPID(double xRequest, double yRequest, double rotateRequest) {
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);

    double velocityOutput = m_tractionControlController.throttleLookup(moveRequest);
    double rotateOutput = m_turnPIDController.calculate(getAngle(), getTurnRate(), rotateRequest);

    drive(velocityOutput * Math.cos(moveDirection), velocityOutput * Math.sin(moveDirection), rotateOutput);
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param moduleStates Calculated swerve module states
   */
  public void autoDrive(SwerveModuleState[] moduleStates) {
    setSwerveModules(moduleStates);
  }

  /**
   * Start calling this repeatedly when robot is in danger of tipping over
   */
  public void antiTip() {
    // Calculate direction of tip
    double direction = Math.atan2(getPitch(), getRoll());

    // Drive to counter tipping motion
    drive(DRIVE_MAX_LINEAR_SPEED / 2 * Math.cos(direction), DRIVE_MAX_LINEAR_SPEED / 2 * Math.sin(direction), 0.0);
  }

  /**
   * Generate AutoTrajectory to drive to pose
   * @param pose Destination pose
   * @return AutoTrajectory that will drive robot to desired pose
   */
  public AutoTrajectory driveToPose(Pose2d pose) {
    Rotation2d heading = new Rotation2d(getPose().getX() - pose.getX(), getPose().getY() - pose.getY());
    List<PathPoint> waypoints = List.of(
      new PathPoint(getPose().getTranslation(), Rotation2d.fromRadians(0.0), getInertialVelocity()),
      new PathPoint(pose.getTranslation(), heading, pose.getRotation())
    );

    return new AutoTrajectory(this, waypoints, DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION);
  }

  /**
   * Lock swerve modules
   */
  public void lock() {
    m_lFrontModule.lock();
    m_rFrontModule.lock();
    m_lRearModule.lock();
    m_rRearModule.lock();
  }

  /**
   * Stop robot
   */
  public void stop() {
    m_lFrontModule.stop();
    m_rFrontModule.stop();
    m_lRearModule.stop();
    m_rRearModule.stop();
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_tractionControlController.toggleTractionControl();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_tractionControlController.enableTractionControl();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_tractionControlController.disableTractionControl();
  }

  /**
   * Reset pose estimator
   * @param pose Pose to set robot to
   */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(m_navx.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Get estimated robot pose
   * @return Currently estimated robot pose
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Get drivetrain kinematics
   * @return Kinematics object
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Get whether or not robot is tipping over
   * @return True if robot is tipping
   */
  public boolean isTipping() {
    return Math.abs(getPitch()) > TIP_THRESHOLD ||
           Math.abs(getRoll()) > TIP_THRESHOLD;
  }


  /**
   * Get whether or not robot is nearly balanced
   * @return True if robot is (nearly) balanced
   */
  public boolean isBalanced() {
    return Math.abs(getPitch()) < BALANCED_THRESHOLD && 
           Math.abs(getRoll()) < BALANCED_THRESHOLD;
  }

  /**
   * Get inertial velocity of robot
   * @return Inertial velocity of robot in m/s
   */
  public double getInertialVelocity() {
    return Math.hypot(m_navx.getVelocityX(), m_navx.getVelocityY());
  }

  /**
   * Get pitch of robot
   * @return Current pitch angle of robot in degrees
   */
  public double getPitch() {
    // Robot pitch axis is navX pitch axis
    return m_navx.getPitch();
  }

  /**
   * Get roll of robot
   * @return Current roll angle of robot in degrees
   */
  public double getRoll() {
    // Robot roll axis is navX roll axis
    return m_navx.getRoll();
  }

  /**
   * Get angle of robot
   * @return Current angle of robot in degrees
   */
  public double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Get rotate rate of robot
   * @return Current rotate rate of robot in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate();
  }

  @Override
  public void close() {
    m_lFrontModule.close();
    m_rFrontModule.close();
    m_lRearModule.close();
    m_rRearModule.close();
    m_navx.close();
  }
}
