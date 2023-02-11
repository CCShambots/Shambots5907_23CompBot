// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.ShamLib.motors.PIDFGains;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1);

  public static final class ControllerConversions{
    public static final double DEADBAND = 0.025;
    public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
  }

  public static final class SwerveDrivetrain {

    //TODO: Remove
    public static final Pose3d limelightPose = new Pose3d(Units.inchesToMeters(-5), 0, Units.inchesToMeters(31), new Rotation3d());

    // Distance between centers of right and left wheels on robot in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    // Distance between front and back wheels on robot in meters
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75);

    // Maximum linear chassis speed in meters per second (MK4 standard modules capable of 4.1)
    public static final double MAX_LINEAR_SPEED = 2;
    public static final double MAX_LINEAR_ACCELERATION = 4;

    // Maximum chassis rotational speed in radians per second
    public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;
    public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
    public static final double MAX_ROT_ACCEL = MAX_ROTATION * 3;

    //TODO: figure out which is which
    public static final Translation2d[] moduleOffsets = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //front left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //back left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), //
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(moduleOffsets);

    public static final double P_HOLDANGLETELE = 3.5; //.5
    public static final double I_HOLDANGLETELE = 0.25;
    public static final double D_HOLDANGLETELE = 0;

    public static final double P_HOLDANGLEAUTO = 5; //4
    public static final double I_HOLDANGLEAUTO = .25; //.25
    public static final double D_HOLDANGLEAUTO = 0;

    public static final double P_HOLDTRANSLATION = 1; //1
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

    public static final int MODULE_1_DRIVE_ID = 11;
    public static final int MODULE_1_TURN_ID = 12;
    public static final int MODULE_1_ENCODER_ID = 11;
    public static final double MODULE_1_OFFSET = 76.7;

    public static final int MODULE_2_DRIVE_ID = 13;
    public static final int MODULE_2_TURN_ID = 14;
    public static final int MODULE_2_ENCODER_ID = 13;
    public static final double MODULE_2_OFFSET = 156.6;

    public static final int MODULE_3_DRIVE_ID = 15;
    public static final int MODULE_3_TURN_ID = 16;
    public static final int MODULE_3_ENCODER_ID = 15;
    public static final double MODULE_3_OFFSET = 59.8;

    public static final int MODULE_4_DRIVE_ID = 17;
    public static final int MODULE_4_TURN_ID = 18;
    public static final int MODULE_4_ENCODER_ID = 17;
    public static final double MODULE_4_OFFSET = -153.1;

    public static Supplier<Pose2d> getOdoPose;
    public static Supplier<Rotation2d> getDrivetrainAngle;
  }

  public static final class SwerveModule {

    // Drive motor PID controller coefficients
    // public static final double P_DRIVE = 0.175;
    public static final double P_DRIVE = 0.0;
    public static final double I_DRIVE = 0;
    public static final double D_DRIVE = 0;
    public static final double KF_DRIVE = 0.0679;

    // Use this ratio to convert from Falcon angular velocity to wheel angular velocity
    public static final double DRIVE_RATIO = 8.14;

    // Turn motor PID controller coefficients
    // public static final double P_TURN = 9;
    public static final double P_TURN = 0.6;
    public static final double I_TURN = 0;
    public static final double D_TURN = 0; //0.1
    public static final double KF_TURN = 0.06464446306847016; //0.15
    public static final double MAX_TURN_SPEED = 50; // Rad/S //50
    public static final double MAX_TURN_ACCEL = 400; // Rad/S^2

    public static final PIDFGains TURN_GAINS = new PIDFGains(
            P_TURN,
            I_TURN,
            D_TURN,
            KF_TURN
    );

    public static final PIDFGains DRIVE_GAINS = new PIDFGains(
            P_DRIVE,
            I_DRIVE,
            D_DRIVE,
            KF_DRIVE
    );

    public static final int kTimeoutMs = 30;//TODO: change these if needed vv
    public static final int kPIDLoopIdx = 0;
    public static final int kSlotIdx = 0;
  }
}
