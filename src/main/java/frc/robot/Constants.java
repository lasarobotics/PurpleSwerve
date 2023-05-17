// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.PIDConstants;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Field {
    public static final double FIELD_WIDTH = 8.1026;
    public static final double FIELD_LENGTH = 16.4846;
  }

  public static class Global {
    public static final double ROBOT_LOOP_PERIOD = 1.0 / 50.0;

    // Motor RPMs, encoder values, and gear ratios
    public static final int NEO_MAX_RPM = 5676;
    public static final int NEO_ENCODER_TICKS_PER_ROTATION = 42;
    public static final int REV_ENCODER_TICKS_PER_ROTATION = 8192;
  }

  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.10;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_TURN_PID = new PIDConstants(5.0, 0.0, 0.8, 0.0);
    public static final double DRIVE_SLIP_RATIO = 0.05;
    public static final double DRIVE_TURN_SCALAR = 45.0;
    public static final double DRIVE_LOOKAHEAD = 3;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 0.042, 0.168, 0.378, 0.672, 1.050, 1.512, 2.508, 2.688, 3.402, 4.200 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.008, 0.032, 0.072, 0.128, 0.200, 0.288, 0.392, 0.512, 0.768, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

    // Swerve velocity PID settings
    private static final double DRIVE_VELOCITY_kP = 0.04;
    private static final double DRIVE_VELOCITY_kI = 0.0;
    private static final double DRIVE_VELOCITY_kD = 0.0;
    private static final double DRIVE_VELOCITY_kF = 1 / (((Global.NEO_MAX_RPM / 60) * DriveSubsystem.DRIVE_WHEEL_DIAMETER_METERS * Math.PI) / DriveSubsystem.DRIVE_GEAR_RATIO);
    private static final double DRIVE_VELOCITY_TOLERANCE = 0.01;
    private static final boolean DRIVE_VELOCITY_SENSOR_PHASE = false;
    private static final boolean DRIVE_INVERT_MOTOR = false;

    // Swerve velocity PID config
    public static final SparkPIDConfig DRIVE_VELOCITY_CONFIG = new SparkPIDConfig(
      DRIVE_VELOCITY_SENSOR_PHASE,
      DRIVE_INVERT_MOTOR,
      DRIVE_VELOCITY_kP,
      DRIVE_VELOCITY_kI,
      DRIVE_VELOCITY_kD,
      DRIVE_VELOCITY_kF,
      DRIVE_VELOCITY_TOLERANCE
    );

    // Swerve rotate PID settings
    private static final double DRIVE_ROTATE_kP = 1.0;
    private static final double DRIVE_ROTATE_kI = 0.0;
    private static final double DRIVE_ROTATE_kD = 0.0;
    private static final double DRIVE_ROTATE_kF = 0.0;
    private static final double DRIVE_ROTATE_TOLERANCE = 0.01;
    private static final double DRIVE_ROTATE_LOWER_LIMIT = 0.0;
    private static final double DRIVE_ROTATE_UPPER_LIMIT = 0.0;
    private static final boolean DRIVE_ROTATE_SOFT_LIMITS = false;
    private static final boolean DRIVE_ROTATE_SENSOR_PHASE = true;
    private static final boolean DRIVE_ROTATE_INVERT_MOTOR = false;

    // Swerve rotate PID config
    public static final SparkPIDConfig DRIVE_ROTATE_CONFIG = new SparkPIDConfig(
      DRIVE_ROTATE_SENSOR_PHASE,
      DRIVE_ROTATE_INVERT_MOTOR,
      DRIVE_ROTATE_kP,
      DRIVE_ROTATE_kI,
      DRIVE_ROTATE_kD,
      DRIVE_ROTATE_kF,
      DRIVE_ROTATE_TOLERANCE,
      DRIVE_ROTATE_LOWER_LIMIT,
      DRIVE_ROTATE_UPPER_LIMIT,
      DRIVE_ROTATE_SOFT_LIMITS
    );
  }

  public static class DriveHardware {
    public static final SparkMax.ID LEFT_FRONT_DRIVE_MOTOR_ID = new SparkMax.ID(2, "LF drive motor");
    public static final SparkMax.ID LEFT_FRONT_ROTATE_MOTOR_ID = new SparkMax.ID(3, "LF rotate motor");
    public static final SparkMax.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new SparkMax.ID(4, "RF drive motor");
    public static final SparkMax.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new SparkMax.ID(5, "RF rotate motor");
    public static final SparkMax.ID LEFT_REAR_DRIVE_MOTOR_ID = new SparkMax.ID(6, "LR drive motor");
    public static final SparkMax.ID LEFT_REAR_ROTATE_MOTOR_ID = new SparkMax.ID(7, "LR rotate motor");
    public static final SparkMax.ID RIGHT_REAR_DRIVE_MOTOR_ID = new SparkMax.ID(8, "RR drive motor");
    public static final SparkMax.ID RIGHT_REAR_ROTATE_MOTOR_ID = new SparkMax.ID(9, "RR rotate motor");

    public static final int LED_STRIP_PORT = 0;
    public static final int LED_STRIP_LENGTH = 200;
  }

  public static class VisionHardware {
    public static final String CAMERA_0_NAME = "camera0";
    public static final Transform3d CAMERA_0_LOCATION = new Transform3d(
      new Translation3d(0.0, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, 0.0)
    );
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }
}