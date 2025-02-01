package frc.robot;

import java.util.Optional;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.child.MAXSwerveModule;
import org.lasarobotics.drive.swerve.child.SwerveX2Module;
import org.lasarobotics.hardware.ctre.CANcoder;
import org.lasarobotics.hardware.ctre.PhoenixCANBus;
import org.lasarobotics.hardware.ctre.TalonFX;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.vision.AprilTagCamera.Resolution;

import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public final class Constants {

  public static class Field {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public static final AprilTag BLUE_SPEAKER = getTag(7).get();
    public static final AprilTag RED_SPEAKER = getTag(4).get();
    public static final AprilTag BLUE_AMP = getTag(6).get();


    /**
     * Get AprilTag from field
     * @param id Tag ID
     * @return AprilTag matching ID
     */
    public static Optional<AprilTag> getTag(int id) {
      return FIELD_LAYOUT.getTags().stream().filter((tag) -> tag.ID == id).findFirst();
    }
  }

  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;
    public static final Dimensionless CONTROLLER_DEADBAND = Units.Percent.of(10);
  }

  
  public static class WiggleStick {
    
    public static final SparkFlexConfig WIGGLE_STICK_CONFIG = new SparkFlexConfig();

    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(10, 20);
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

  public static class Drive {
    public static enum Vendor {
      REV,
      CTRE;
    }
    public static final Vendor DRIVE_VENDOR = Vendor.REV;
    public static final DriveWheel DRIVE_WHEEL = DriveWheel.create(Units.Inches.of(4.0), Units.Value.of(1.3), Units.Value.of(1.2));
    public static final PIDConstants DRIVE_PID = PIDConstants.of(0.3, 0.0, 0.001, 0.0, 0.0);
    public static final FFConstants DRIVE_FF = FFConstants.of(0.2, 0.0, 0.0, 0.0);
    public static final PIDConstants ROTATE_PID = PIDConstants.of(2.0, 0.0, 0.1, 0.0, 0.0);
    public static final FFConstants ROTATE_FF = FFConstants.of(0.2, 0.0, 0.0, 0.0);
    public static final PIDConstants DRIVE_ROTATE_PID = PIDConstants.of(8.0, 0.0, 0.3, 0.0, 0.0);
    public static final PIDConstants DRIVE_AUTO_AIM_PID = PIDConstants.of(12.0, 0.0, 0.1, 0.0, 0.0);
    public static final Dimensionless DRIVE_SLIP_RATIO = Units.Percent.of(3.0);
    public static final Angle DRIVE_TURN_SCALAR = Units.Degrees.of(90.0);
    public static final Time DRIVE_LOOKAHEAD = Units.Seconds.of(0.2);

    public static final Distance DRIVE_WHEELBASE = Units.Meters.of(0.5588);
    public static final Distance DRIVE_TRACK_WIDTH = Units.Meters.of(0.5588);
    public static final Mass MASS = Units.Pounds.of(110.0);
    public static final Time AUTO_LOCK_TIME = Units.Seconds.of(3.0);
    public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(90.0);

    public static final ControlCentricity DRIVE_CONTROL_CENTRICITY = ControlCentricity.FIELD_CENTRIC;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 0.052, 0.207, 0.465, 0.827, 1.293, 1.862, 2.534, 3.310, 4.189, 5.172 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.010, 0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.600, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

    public static final SwerveX2Module.GearRatio CTRE_GEAR_RATIO = SwerveX2Module.GearRatio.X4_3;
    public static final MAXSwerveModule.GearRatio REV_GEAR_RATIO = MAXSwerveModule.GearRatio.L3;

    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(DRIVE_WHEEL.diameter.div(2), Units.MetersPerSecond.of(5.172), 1.3, DCMotor.getKrakenX60Foc(1), DRIVE_CURRENT_LIMIT, 1);
  }

  public static class CTREDriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final TalonFX.ID LEFT_FRONT_DRIVE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/LeftFront/Drive", PhoenixCANBus.CANIVORE, 2);
    public static final TalonFX.ID LEFT_FRONT_ROTATE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/LeftFront/Rotate", PhoenixCANBus.CANIVORE, 3);
    public static final CANcoder.ID LEFT_FRONT_ROTATE_ENCODER_ID = new CANcoder.ID("DriveHardware/Swerve/LeftFront/Encoder", PhoenixCANBus.CANIVORE, 10);
    public static final TalonFX.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/RightFront/Drive", PhoenixCANBus.CANIVORE, 4);
    public static final TalonFX.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/RightFront/Rotate", PhoenixCANBus.CANIVORE, 5);
    public static final CANcoder.ID RIGHT_FRONT_ROTATE_ENCODER_ID = new CANcoder.ID("DriveHardware/Swerve/RightFront/Encoder", PhoenixCANBus.CANIVORE, 11);
    public static final TalonFX.ID LEFT_REAR_DRIVE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/LeftRear/Drive", PhoenixCANBus.CANIVORE, 6);
    public static final TalonFX.ID LEFT_REAR_ROTATE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/LeftRear/Rotate", PhoenixCANBus.CANIVORE, 7);
    public static final CANcoder.ID LEFT_REAR_ROTATE_ENCODER_ID = new CANcoder.ID("DriveHardware/Swerve/LeftRear/Encoder", PhoenixCANBus.CANIVORE, 12);
    public static final TalonFX.ID RIGHT_REAR_DRIVE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/RightRear/Drive", PhoenixCANBus.CANIVORE, 8);
    public static final TalonFX.ID RIGHT_REAR_ROTATE_MOTOR_ID = new TalonFX.ID("DriveHardware/Swerve/RightRear/Rotate", PhoenixCANBus.CANIVORE, 9);
    public static final CANcoder.ID RIGHT_REAR_ROTATE_ENCODER_ID = new CANcoder.ID("DriveHardware/Swerve/RightRear/Encoder", PhoenixCANBus.CANIVORE, 13);
  }

  public static class REVDriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 2);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 3);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 4);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 5);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 6);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 7);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 8);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 9);
  }

  public static class VisionHardware {
    public static final String CAMERA_A_NAME = "Arducam_OV9782_USB_Camera_A";
    public static final Transform3d CAMERA_A_LOCATION = new Transform3d(
      new Translation3d(-0.1016, -0.2921, 0.521),
      new Rotation3d(0.0, Math.toRadians(-26.0), Math.toRadians(+180.0))
    );
    public static final Resolution CAMERA_A_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_A_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_B_NAME = "Arducam_OV9782_USB_Camera_B";
    public static final Transform3d CAMERA_B_LOCATION = new Transform3d(
      new Translation3d(0.0254, -0.2921, 0.584),
      new Rotation3d(0.0, Math.toRadians(-25.6), 0.0)
    );
    public static final Resolution CAMERA_B_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_B_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_OBJECT_NAME = "Arducam_OV9782_USB_Camera_C";
    public static final Transform3d CAMERA_OBJECT_LOCATION = new Transform3d(
      new Translation3d(0.3, 0.0, 0.5),
      new Rotation3d(0, Math.toRadians(+15.0), Math.toRadians(180))
    );
    public static final Resolution CAMERA_OBJECT_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_OBJECT_FOV = Rotation2d.fromDegrees(79.7);
  }
}
