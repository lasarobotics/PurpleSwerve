// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.swerve.SwerveDrive;
import org.lasarobotics.drive.swerve.child.SwerveX2Module;
import org.lasarobotics.drive.swerve.parent.CTRESwerveModule;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.vision.AprilTagCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class CTREDriveSubsystem extends SwerveDrive {

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param rotatePIDF PID gains for rotate PID
   * @param aimPIDF PID gains for aim PID
   * @param controlCentricity Control centricity
   * @param throttleInputCurve Spline function characterising throttle input
   * @param rotateInputCurve Spline function characterising rotate input
   * @param rotateScalar Scalar for rotate input
   * @param deadband Deadband for controller input [+0.001, +0.2]
   * @param lookAhead Rotate PID lookahead
   */
  public CTREDriveSubsystem(Hardware drivetrainHardware, PIDConstants rotatePIDF, PIDConstants aimPIDF, ControlCentricity controlCentricity,
                        PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction rotateInputCurve,
                        Angle rotateScalar, Dimensionless deadband, Time lookAhead) {
    super(drivetrainHardware, rotatePIDF, aimPIDF, controlCentricity, throttleInputCurve, rotateInputCurve, rotateScalar, deadband, lookAhead);
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    NavX2 navx = new NavX2(Constants.CTREDriveHardware.NAVX_ID);

    CTRESwerveModule lFrontModule = SwerveX2Module.create(
      CTRESwerveModule.initializeHardware(
        Constants.CTREDriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID,
        Constants.CTREDriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID,
        Constants.CTREDriveHardware.LEFT_FRONT_ROTATE_ENCODER_ID
      ),
      org.lasarobotics.drive.swerve.SwerveModule.Location.LeftFront,
      org.lasarobotics.drive.swerve.SwerveModule.MountOrientation.INVERTED,
      Constants.Drive.CTRE_GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_PID,
      Constants.Drive.DRIVE_FF,
      Constants.Drive.ROTATE_PID,
      Constants.Drive.ROTATE_FF,
      Constants.Drive.DRIVE_SLIP_RATIO,
      Constants.Drive.MASS,
      Constants.Drive.DRIVE_WHEELBASE,
      Constants.Drive.DRIVE_TRACK_WIDTH,
      Constants.Drive.AUTO_LOCK_TIME,
      Constants.Drive.DRIVE_CURRENT_LIMIT
    );

    CTRESwerveModule rFrontModule = SwerveX2Module.create(
      CTRESwerveModule.initializeHardware(
        Constants.CTREDriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID,
        Constants.CTREDriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID,
        Constants.CTREDriveHardware.RIGHT_FRONT_ROTATE_ENCODER_ID
      ),
      org.lasarobotics.drive.swerve.SwerveModule.Location.LeftFront,
      org.lasarobotics.drive.swerve.SwerveModule.MountOrientation.INVERTED,
      Constants.Drive.CTRE_GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_PID,
      Constants.Drive.DRIVE_FF,
      Constants.Drive.ROTATE_PID,
      Constants.Drive.ROTATE_FF,
      Constants.Drive.DRIVE_SLIP_RATIO,
      Constants.Drive.MASS,
      Constants.Drive.DRIVE_WHEELBASE,
      Constants.Drive.DRIVE_TRACK_WIDTH,
      Constants.Drive.AUTO_LOCK_TIME,
      Constants.Drive.DRIVE_CURRENT_LIMIT
    );

    CTRESwerveModule lRearModule = SwerveX2Module.create(
      CTRESwerveModule.initializeHardware(
        Constants.CTREDriveHardware.LEFT_REAR_DRIVE_MOTOR_ID,
        Constants.CTREDriveHardware.LEFT_REAR_ROTATE_MOTOR_ID,
        Constants.CTREDriveHardware.LEFT_REAR_ROTATE_ENCODER_ID
      ),
      org.lasarobotics.drive.swerve.SwerveModule.Location.LeftFront,
      org.lasarobotics.drive.swerve.SwerveModule.MountOrientation.INVERTED,
      Constants.Drive.CTRE_GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_PID,
      Constants.Drive.DRIVE_FF,
      Constants.Drive.ROTATE_PID,
      Constants.Drive.ROTATE_FF,
      Constants.Drive.DRIVE_SLIP_RATIO,
      Constants.Drive.MASS,
      Constants.Drive.DRIVE_WHEELBASE,
      Constants.Drive.DRIVE_TRACK_WIDTH,
      Constants.Drive.AUTO_LOCK_TIME,
      Constants.Drive.DRIVE_CURRENT_LIMIT
    );

    CTRESwerveModule rRearModule = SwerveX2Module.create(
      CTRESwerveModule.initializeHardware(
        Constants.CTREDriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID,
        Constants.CTREDriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID,
        Constants.CTREDriveHardware.RIGHT_REAR_ROTATE_ENCODER_ID
      ),
      org.lasarobotics.drive.swerve.SwerveModule.Location.LeftFront,
      org.lasarobotics.drive.swerve.SwerveModule.MountOrientation.INVERTED,
      Constants.Drive.CTRE_GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_PID,
      Constants.Drive.DRIVE_FF,
      Constants.Drive.ROTATE_PID,
      Constants.Drive.ROTATE_FF,
      Constants.Drive.DRIVE_SLIP_RATIO,
      Constants.Drive.MASS,
      Constants.Drive.DRIVE_WHEELBASE,
      Constants.Drive.DRIVE_TRACK_WIDTH,
      Constants.Drive.AUTO_LOCK_TIME,
      Constants.Drive.DRIVE_CURRENT_LIMIT
    );

    AprilTagCamera frontCamera = new AprilTagCamera(
      Constants.VisionHardware.CAMERA_A_NAME,
      Constants.VisionHardware.CAMERA_A_LOCATION,
      Constants.VisionHardware.CAMERA_A_RESOLUTION,
      Constants.VisionHardware.CAMERA_A_FOV,
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
    );

    AprilTagCamera rearCamera = new AprilTagCamera(
      Constants.VisionHardware.CAMERA_B_NAME,
      Constants.VisionHardware.CAMERA_B_LOCATION,
      Constants.VisionHardware.CAMERA_B_RESOLUTION,
      Constants.VisionHardware.CAMERA_B_FOV,
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
    );

    Hardware drivetrainHardware = new Hardware(navx, lFrontModule, rFrontModule, lRearModule, rRearModule, frontCamera, rearCamera);

    return drivetrainHardware;
  }
}