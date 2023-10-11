// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.led.LEDStrip;
import frc.robot.subsystems.led.LEDStrip.Pattern;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.NavX2;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    NavX2 navx;
    MAXSwerveModule lFrontModule;
    MAXSwerveModule rFrontModule;
    MAXSwerveModule lRearModule;
    MAXSwerveModule rRearModule;
    LEDStrip ledStrip;

    public Hardware(NavX2 navx,
                    MAXSwerveModule lFrontModule,
                    MAXSwerveModule rFrontModule,
                    MAXSwerveModule lRearModule,
                    MAXSwerveModule rRearModule,
                    LEDStrip ledStrip) {
      this.navx = navx;
      this.lFrontModule = lFrontModule;
      this.rFrontModule = rFrontModule;
      this.lRearModule = lRearModule;
      this.rRearModule = rRearModule;
      this.ledStrip = ledStrip;
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

  private ThrottleMap m_throttleMap;
  private TurnPIDController m_turnPIDController;
  private ProfiledPIDController m_autoAimPIDController;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private AdvancedSwerveKinematics m_advancedKinematics;

  private NavX2 m_navx;
  private MAXSwerveModule m_lFrontModule;
  private MAXSwerveModule m_rFrontModule;
  private MAXSwerveModule m_lRearModule;
  private MAXSwerveModule m_rRearModule;
  private LEDStrip m_ledStrip;

  private final double TOLERANCE = 0.125;
  private final double TIP_THRESHOLD = 30.0;
  private final double BALANCED_THRESHOLD = 5.0;
  private final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1));
  private final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(40));
  private final TrapezoidProfile.Constraints AIM_PID_CONSTRAINT = new TrapezoidProfile.Constraints(1080.0, 1440.0);

  private final String POSE_LOG_ENTRY = "Pose"; 
  private final String SWERVE_STATE_LOG_ENTRY = "Swerve";

  private ChassisSpeeds m_desiredChassisSpeeds;
  private Field2d m_field;
  private NetworkTable m_table;
  private DoubleArrayPublisher m_posePublisher;

  private boolean m_isTractionControlEnabled = true;
  private Pose2d m_previousPose;
  private Rotation2d m_currentHeading;

  public final Command ANTI_TIP_COMMAND = new FunctionalCommand(
    () -> m_ledStrip.set(Pattern.RED_STROBE),
    () -> antiTip(),
    (interrupted) -> {
      m_ledStrip.set(Pattern.GREEN_SOLID);
      resetTurnPID();
      lock();
      stop();
      m_ledStrip.set(Pattern.TEAM_COLOR_SOLID);
    },
    this::isBalanced,
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
   * @param slipRatio Traction control slip ratio [+0.01, +0.15]
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   */
  public DriveSubsystem(Hardware drivetrainHardware, double kP, double kD,
                        double turnScalar, double deadband, double lookAhead, double slipRatio,
                        PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve) {
    setSubsystem(getClass().getSimpleName());
    m_throttleMap = new ThrottleMap(throttleInputCurve, deadband, DRIVE_MAX_LINEAR_SPEED);
    m_turnPIDController = new TurnPIDController(kP, kD, turnScalar, deadband, lookAhead, turnInputCurve);

    this.m_navx = drivetrainHardware.navx;
    this.m_lFrontModule = drivetrainHardware.lFrontModule;
    this.m_rFrontModule = drivetrainHardware.rFrontModule;
    this.m_lRearModule = drivetrainHardware.lRearModule;
    this.m_rRearModule = drivetrainHardware.rRearModule;
    this.m_ledStrip = drivetrainHardware.ledStrip;

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

    // Define advanced drivetrain kinematics
    m_advancedKinematics = new AdvancedSwerveKinematics(true,
                                                        m_lFrontModule.getModuleCoordinate(),
                                                        m_rFrontModule.getModuleCoordinate(),
                                                        m_lRearModule.getModuleCoordinate(),
                                                        m_rRearModule.getModuleCoordinate());

    // Initialise pose estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(getAngle()),
      getModulePositions(), 
      new Pose2d(),
      ODOMETRY_STDDEV,
      VISION_STDDEV
    );

    // Initialise chassis speeds
    m_desiredChassisSpeeds = new ChassisSpeeds();

    // Setup anti-tip command
    new Trigger(this::isTipping).onTrue(ANTI_TIP_COMMAND);

    // Register LED strip with LED subsystem
    LEDSubsystem.getInstance().add(m_ledStrip);

    // Set LED strip to team color
    m_ledStrip.set(Pattern.TEAM_COLOR_SOLID);

    // Initialise field
    m_field = new Field2d();

    // Setup auto-aim PID controller
    m_autoAimPIDController = new ProfiledPIDController(kP, 0.0, kD, AIM_PID_CONSTRAINT, Constants.Global.ROBOT_LOOP_PERIOD);
    m_autoAimPIDController.enableContinuousInput(-180.0, +180.0);

    // Initialise other variables
    m_previousPose = new Pose2d();
    m_currentHeading = new Rotation2d();

    // Setup NetworkTables
    m_table = NetworkTableInstance.getDefault().getTable(getName());
    m_posePublisher = m_table.getDoubleArrayTopic(POSE_LOG_ENTRY).publish();
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    NavX2 navx = new NavX2(Constants.DriveHardware.NAVX_ID, Constants.Global.ROBOT_LOOP_HZ);

    MAXSwerveModule lFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.LeftFront, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_MAX_LINEAR_SPEED,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    MAXSwerveModule rFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.RightFront, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_MAX_LINEAR_SPEED,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    MAXSwerveModule lRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.LeftRear, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_MAX_LINEAR_SPEED,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    MAXSwerveModule rRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.RightRear, 
      Constants.Drive.DRIVE_VELOCITY_CONFIG, 
      Constants.Drive.DRIVE_ROTATE_CONFIG,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_MAX_LINEAR_SPEED,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      DRIVE_GEAR_RATIO,
      DRIVE_WHEEL_DIAMETER_METERS
    );

    LEDStrip ledStrip = new LEDStrip(LEDStrip.initializeHardware(Constants.DriveHardware.LED_STRIP_ID));

    Hardware drivetrainHardware = new Hardware(navx, lFrontModule, rFrontModule, lRearModule, rRearModule, ledStrip);

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
   * Set swerve modules, automatically applying traction control
   * @param moduleStates Array of calculated module states
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param rotateRate Current rotate rate (degrees/s)
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates, double inertialVelocity, double rotateRate) {
    m_lFrontModule.set(moduleStates, inertialVelocity, rotateRate);
    m_rFrontModule.set(moduleStates, inertialVelocity, rotateRate);
    m_lRearModule.set(moduleStates, inertialVelocity, rotateRate);
    m_rRearModule.set(moduleStates, inertialVelocity, rotateRate);
  }

  /**
   * Drive robot and apply traction control
   * @param xRequest Desired X (forward) velocity (m/s)
   * @param yRequest Desired Y (sideways) velocity (m/s)
   * @param rotateRequest Desired rotate rate (degrees/s)
   * @param inertialVelocity Current robot inertial velocity (m/s)
   * @param rotateRate Current robot rotate rate (degrees/s)
   */
  private void drive(double xRequest, double yRequest, double rotateRequest, double inertialVelocity, double rotateRate) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, Math.toRadians(rotateRequest))
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation()
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITH traction control
    setSwerveModules(moduleStates, inertialVelocity, rotateRate);
  }

  /**
   * Drive robot without traction control
   * @param xRequest Desired X (forward) velocity (m/s)
   * @param yRequest Desired Y (sideways) velocity (m/s)
   * @param rotateRequest Desired rotate rate (degrees/s)
   */
  private void drive(double xRequest, double yRequest, double rotateRequest) {
     // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, Math.toRadians(rotateRequest))
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation()
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);
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
   * Get current module states
   * @return Array of swerve module states
   */
  private SwerveModuleState[] getModuleStates() {
     return new SwerveModuleState[] {
      m_lFrontModule.getState(),
      m_rFrontModule.getState(),
      m_lRearModule.getState(),
      m_rRearModule.getState()
    };
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
    // Save previous pose
    m_previousPose = getPose();

    // Update pose based on odometry
    m_poseEstimator.update(getRotation2d(), getModulePositions());

    // Update current heading
    m_currentHeading = new Rotation2d(getPose().getX() - m_previousPose.getX(), getPose().getY() - m_previousPose.getY());

    // Skip vision pose estimation if running in simulation
    if (RobotBase.isSimulation()) return;

    // Get estimated poses from VisionSubsystem
    var visionEstimatedRobotPoses = VisionSubsystem.getInstance().getEstimatedGlobalPose();

    // Exit if no valid vision pose estimates
    if (visionEstimatedRobotPoses.isEmpty()) return;

    // Add vision measurements to pose estimator
    for (var visionEstimatedRobotPose : visionEstimatedRobotPoses)
      m_poseEstimator.addVisionMeasurement(visionEstimatedRobotPose.estimatedPose.toPose2d(), visionEstimatedRobotPose.timestampSeconds);
  }

  /**
   * Log DriveSubsystem outputs
   */
  private void logOutputs() {
    Logger.getInstance().recordOutput(String.join("/", getName(), POSE_LOG_ENTRY), getPose());
    Logger.getInstance().recordOutput(String.join("/", getName(), SWERVE_STATE_LOG_ENTRY), getModuleStates());
  }

  /**
   * SmartDashboard indicators
   */
  public void smartDashboard() {
    SmartDashboard.putBoolean("TC", m_isTractionControlEnabled);
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Publish data to network tables
   */
  private void publishData() {
    Pose2d currentPose = getPose();
    double[] poseArray = { currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees() };
    m_posePublisher.set(poseArray);
    m_field.setRobotPose(currentPose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_navx.periodic();
    m_lFrontModule.periodic();
    m_rFrontModule.periodic();
    m_lRearModule.periodic();
    m_rRearModule.periodic();

    updatePose();
    smartDashboard();
    publishData();
    logOutputs();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    double angle = m_navx.getSimAngle() + Math.toDegrees(m_desiredChassisSpeeds.omegaRadiansPerSecond) * Constants.Global.ROBOT_LOOP_PERIOD;
    m_navx.setSimAngle(angle);
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

    double velocityOutput = m_throttleMap.throttleLookup(moveRequest);
    double rotateOutput = m_turnPIDController.calculate(getAngle(), getRotateRate(), rotateRequest);

    m_autoAimPIDController.calculate(getPose().getRotation().getDegrees(), getPose().getRotation().getDegrees());

    drive(velocityOutput * Math.cos(moveDirection), velocityOutput * Math.sin(moveDirection), rotateOutput, getInertialVelocity(), getRotateRate());
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
    double direction = Math.atan2(getRoll(), getPitch());

    // Drive to counter tipping motion
    drive(DRIVE_MAX_LINEAR_SPEED / 4 * Math.cos(direction), DRIVE_MAX_LINEAR_SPEED / 4 * Math.sin(direction), 0.0);
  }

  /**
   * Generate AutoTrajectory to drive to pose
   * @param pose Destination pose
   * @return AutoTrajectory that will drive robot to desired pose
   */
  public AutoTrajectory driveToPose(Pose2d pose) {
    Pose2d currentPose = getPose();
    Rotation2d heading = new Rotation2d(currentPose.getX() - pose.getX(), currentPose.getY() - pose.getY());
    List<PathPoint> waypoints = List.of(
      new PathPoint(currentPose.getTranslation(), Rotation2d.fromRadians(0.0), getInertialVelocity()),
      new PathPoint(pose.getTranslation(), heading, pose.getRotation())
    );

    return new AutoTrajectory(this, waypoints, DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION);
  }

  /**
   * Aim robot at a desired point on the field
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param point Target point
   */
  public void aimAtPoint(double xRequest, double yRequest, Translation2d point) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    double velocityOutput = m_throttleMap.throttleLookup(moveRequest);

    // Get current pose
    Pose2d currentPose = getPose();
    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
    // Movement vector of robot
    Vector2D robotVector = new Vector2D(velocityOutput * m_currentHeading.getCos(), velocityOutput * m_currentHeading.getSin());
    // Aim point
    Translation2d aimPoint = point.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Vector2D targetVector = new Vector2D(currentPose.getTranslation().getDistance(point) * targetAngle.getCos(), currentPose.getTranslation().getDistance(point) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Vector2D parallelRobotVector = targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
    // Perpendicular component of robot's motion to target vector
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    double rotateOutput = m_autoAimPIDController.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

    Logger.getInstance().recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));

    // Drive robot accordingly
    drive(velocityOutput * Math.cos(moveDirection), velocityOutput * Math.sin(moveDirection), -rotateOutput);
  }

  /**
   * Aim robot at a desired point on the field (without any strafing)
   * @param point Target point
   */
  public void aimAtPoint(Translation2d point) {
    aimAtPoint(0.0, 0.0, point);
  }

  /**
   * Reset DriveSubsystem turn PID
   */
  public void resetTurnPID() {
    m_turnPIDController.setSetpoint(getAngle());
    m_turnPIDController.reset();
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
    m_isTractionControlEnabled = !m_isTractionControlEnabled;
    m_lFrontModule.toggleTractionControl();
    m_rFrontModule.toggleTractionControl();
    m_lRearModule.toggleTractionControl();
    m_rRearModule.toggleTractionControl();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_isTractionControlEnabled = true;
    m_lFrontModule.enableTractionControl();
    m_rFrontModule.enableTractionControl();
    m_lRearModule.enableTractionControl();
    m_rRearModule.enableTractionControl();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_isTractionControlEnabled = false;
    m_lFrontModule.disableTractionControl();
    m_rFrontModule.disableTractionControl();
    m_lRearModule.disableTractionControl();
    m_rRearModule.disableTractionControl();
  }

  /**
   * Reset pose estimator
   * @param pose Pose to set robot to
   */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(
      getRotation2d(),
      getModulePositions(),
      pose
    );
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
    return Math.hypot(m_navx.getInputs().xVelocity, m_navx.getInputs().yVelocity);
  }

  /**
   * Get pitch of robot
   * @return Current pitch angle of robot in degrees
   */
  public double getPitch() {
    // Robot pitch axis is navX pitch axis
    return m_navx.getInputs().pitchAngle;
  }

  /**
   * Get roll of robot
   * @return Current roll angle of robot in degrees
   */
  public double getRoll() {
    // Robot roll axis is navX roll axis
    return m_navx.getInputs().rollAngle;
  }

  /**
   * Return the heading of the robot in degrees
   * @return Current heading of the robot in degrees
   */
  public double getAngle() {
    return m_navx.getInputs().yawAngle;
  }

  /**
   * Get rotate rate of robot
   * @return Current rotate rate of robot (degrees/s)
   */
  public double getRotateRate() {
    return m_navx.getInputs().yawRate;
  }

  /**
   * Return the heading of the robot as a Rotation2d.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * @return Current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-m_navx.getInputs().yawAngle);
  }

  @Override
  public void close() {
    m_lFrontModule.close();
    m_rFrontModule.close();
    m_lRearModule.close();
    m_rRearModule.close();
    m_navx.close();
    m_posePublisher.close();
  }
}
