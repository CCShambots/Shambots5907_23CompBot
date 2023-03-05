// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.motors.pro.PIDSVGains;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.ShamLib.motors.v5.PIDFGains;
import frc.robot.util.math.Range;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

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
    public static final int SOLENOID_ID_1 = 2;
    public static final int SOLENOID_ID_2 = 3;

    public static final boolean SOLENOID_CLAW_OPEN_STATE = false; //TODO: find value
  }


  public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1);

  public static final class ControllerConversions{
    public static final double DEADBAND = 0.025;
    public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
  }

  public static final class SwerveDrivetrain {

    public static final Pose3d limelightPose = new Pose3d(0, Units.inchesToMeters(12.25), Units.inchesToMeters(5.75), new Rotation3d(0, 0, toRadians(90)));

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
    public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * PI;
    public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * PI);
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
            10, //10
            0,
            0,
            0.3, //0.3
            0.121057 //0.112
    );

    //Drive motor coefficients
    public static final PIDSVGains DRIVE_GAINS = new PIDSVGains(
            0.25, 
            0,
            0,
            0.35, 
            .11066 
    );
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

  public static class Arm {
    //Physial constants for arm dimensions
    public static final double baseToTurret = inchesToMeters(5.5 + 1.44);//Distance from the floor to the top of the turret plate
    public static final double turretToArm = inchesToMeters(14.5); //Distance from the turret to the arm (when the elevator is at 0)
    public static final double armToWrist = inchesToMeters(28.765564);
    public static final double wristToEndEffector = inchesToMeters(12.9016);

    //Turret hardawre details
    public static final int TURRET_ID = 21;
    public static final double TURRET_INPUT_TO_OUTPUT =
            (1.0/ 25.0) * //TODO: Gear ratio on motor
            (10.0 / 140.0) *
            2 * PI //To radians
    ; //Rotations --> Radians
    public static final int TURRET_POT_PORT = 0; //Analog port
    public static final double TURRET_POT_RATIO = 514.2857142857143; //Converts turns of the potentiometer to output degrees
    public static final double TURRET_ENCODER_OFFSET = -252.9; //Degrees
    public static final double TURRET_MAX_VEL = 1000;
    public static final double TURRET_MAX_ACCEL = 1000;
    public static final Range turretRange = Range.fromDegrees(-90, 90);

    //Elevator hardware details
    public static final int ELEVATOR_ID = 22;
    public static final double ELEVATOR_INPUT_TO_OUTPUT =
                (1.0 / 35.0) * //Gearbox
                // (54.0 / 18.0) * //Actual gears
                1.273 * PI * //Pitch diameter (1.273") --> distance traveled by chain (in inches)
                0.0254 //inches to meters
            ; //Converts motor revolutions to meters
    public static final Range elevatorRange = new Range(0, inchesToMeters(26));
    public static final double ELEVATOR_MAX_VEL = 5000; //rot/sec
    public static final double ELEVATOR_MAX_ACCEL = 5000; //rot/sec^2

    //Shoulder hardware details
    public static final int SHOULDER_ID = 23;
    public static final double SHOULDER_INPUT_TO_OUTPUT = (1.0/100.0) * (2.0 / 3.0) * 2 * PI; //Rotations --> Radians
    public static final int SHOULDER_ENCODER_PORT = 8;
    public static final double SHOULDER_ENCODER_OFFSET = 110.983395; //Degrees
    public static final Range shoulderRange = Range.fromDegrees(-35, 90); //TODO
    public static final double SHOULDER_MAX_VEL = toRadians(90); //Radians/sec
    public static final double SHOULDER_MAX_ACCEL = toRadians(90); //Radians/sec^2

    //Wrist hardware details
    public static final int WRIST_ID = 24;
    public static final double WRIST_INPUT_TO_OUTPUT = (1.0 / 49.0) * 2 * PI; //Ticks --> Radians
    public static final int WRIST_ENCODER_PORT = 9;
    public static final double WRIST_ENCODER_OFFSET = 93.733337; //Degrees
    public static final Range wristRange = Range.fromDegrees(-130, 0); //Degrees //TODO:
    public static final double WRIST_MAX_VEL = toRadians(180); //Radians/sec
    public static final double WRIST_MAX_ACCEL = toRadians(180); //Radians/sec^2

    //Rotator hardware details
    public static final int ROTATOR_ID = 25;
    public static final double ROTATOR_ENCODER_OFFSET = toRadians(48.112586); //Radians //TODO
    public static final Range rotatorRange = Range.fromDegrees(-180, 180);

    //PID gains
    public static final PIDSVGains TURRET_GAINS = new PIDSVGains(10, 0, 0, 0.35, 0.114);
    public static final PIDSVGains ELEVATOR_GAINS = new PIDSVGains(2, 0, 0, 0.3, 0.116);
    public static final PIDSVGains SHOULDER_GAINS = new PIDSVGains(10, 0, 0, 0.4, .234);
    public static final PIDGains SHOULDER_CONT_GAINS = new PIDGains(0, 0, 0);
    public static final PIDSVGains WRIST_GAINS = new PIDSVGains(.35, 0, 0, 0, 0.109056); 
    public static final PIDGains WRIST_CONT_GAINS = new PIDGains(5, 0, 0);

    public static final PIDFGains ROTATOR_GAINS = new PIDFGains(2.5, 0, 100, 0);

    //Other constants
    public static final double END_TOLERANCE_CONE_ANGLE = toRadians(2); //Radians



  }
}
