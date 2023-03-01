// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.ShamLib.motors.pro.PIDSVGains;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import static edu.wpi.first.math.util.Units.inchesToMeters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Claw {
    public static final int COMPRESSOR_ID = 1;
    public static final int SOLENOID_ID = 2;

    public static final boolean SOLENOID_CLAW_OPEN_STATE = false; //TODO: find value
  }


  public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1);

  public static final class ControllerConversions{
    public static final double DEADBAND = 0.025;
    public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
  }

  public static final class SwerveDrivetrain {

    public static final Pose3d limelightPose = new Pose3d(0, Units.inchesToMeters(12.25), Units.inchesToMeters(5.75), new Rotation3d(0, 0, Math.toRadians(90)));

    // Distance between centers of right and left wheels on robot in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    // Distance between front and back wheels on robot in meters
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75);

    // Maximum linear chassis speed in meters per second (MK4 standard modules capable of 4.1)
    public static final double MAX_LINEAR_SPEED = 3;
    public static final double MAX_LINEAR_ACCELERATION = 6;

    public static final double MAX_LINEAR_SPEED_AUTO = 1.5;
    public static final double MAX_LINEAR_ACCELERATION_AUTO = 5;

    // Maximum chassis rotational speed in radians per second
    public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;
    public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
    public static final double MAX_ROT_ACCEL = MAX_ROTATION * 3;

    public static final Translation2d[] moduleOffsets = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //front left
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), //back left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), //back right
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2) //front right
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(moduleOffsets);

    public static final double P_HOLDANGLETELE = 0.25;
    public static final double I_HOLDANGLETELE = 0;
    public static final double D_HOLDANGLETELE = 0;

    public static final double P_HOLDANGLEAUTO = 5; 
    public static final double I_HOLDANGLEAUTO = 0; 
    public static final double D_HOLDANGLEAUTO = 0;

    public static final double P_HOLDTRANSLATION = 3;
    public static final double I_HOLDTRANSLATION = 0;
    public static final double D_HOLDTRANSLATION = 0;

    public static final SwerveModuleState STOPPED_STATE = new SwerveModuleState(0, new Rotation2d());

    public static final SwerveModuleState[] X_SHAPE_ARRAY = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
    };

    public static final int PIGEON_ID = 01;

    //front left
    public static final int MODULE_1_DRIVE_ID = 11;
    public static final int MODULE_1_TURN_ID = 12;
    public static final int MODULE_1_ENCODER_ID = 11;
    public static final double MODULE_1_OFFSET = 45.97;

    //back left
    public static final int MODULE_2_DRIVE_ID = 13;
    public static final int MODULE_2_TURN_ID = 14;
    public static final int MODULE_2_ENCODER_ID = 13;
    public static final double MODULE_2_OFFSET = -69.79;

    //back right
    public static final int MODULE_3_DRIVE_ID = 15;
    public static final int MODULE_3_TURN_ID = 16;
    public static final int MODULE_3_ENCODER_ID = 15;
    public static final double MODULE_3_OFFSET = -135.7;

    //front right
    public static final int MODULE_4_DRIVE_ID = 17;
    public static final int MODULE_4_TURN_ID = 18;
    public static final int MODULE_4_ENCODER_ID = 17;
    public static final double MODULE_4_OFFSET = -139.48;

    public static Supplier<Pose2d> getOdoPose;
    public static Supplier<Rotation2d> getDrivetrainAngle;
  }

  public static final class SwerveModule {

    public static final double MAX_TURN_SPEED = 1000; // motor rots / sec
    public static final double MAX_TURN_ACCEL = 1000; // motor rots / sec^2

    //Turn motor coefficients
    public static final PIDSVGains TURN_GAINS = new PIDSVGains(
            10,
            0,
            0,
            0.3,
            0.112
    );

    //Drive motor coefficients
    public static final PIDSVGains DRIVE_GAINS = new PIDSVGains(
            .25,
            0,
            0,
            0.25,
            0.112
    );

    public static final int kTimeoutMs = 30;//TODO: change these if needed vv
    public static final int kPIDLoopIdx = 0;
    public static final int kSlotIdx = 0;
  }

  public static final class Vision {
    public static Pose3d BASE_LIMELIGHT_POSE = new Pose3d(inchesToMeters(-5), 0, inchesToMeters(31), new Rotation3d());

    //Base
    public static final int APRIL_TAG_PIPELINE = 0;

    //Claw
    public static final int CONE_DETECTOR_PIPELINE = 0;
    public static final int CUBE_DETECTOR_PIPELINE = 1;
    public static final int CONE_ORIENTATION_PIPELINE = 2;
  }
}
