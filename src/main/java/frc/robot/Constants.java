// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.motors.pro.PIDSVGains;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import com.ctre.phoenix.led.*;
import frc.robot.ShamLib.Candle.RGB;

import static com.ctre.phoenix.led.LarsonAnimation.BounceMode.Front;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.ShamLib.motors.v5.PIDFGains;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.math.Range;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

public final class Constants {

  public static Alliance alliance = Alliance.Red;
  public static boolean overrideAlliance = false; //Flag to indicate that the drivers have manually set the allianc

  public static final class Claw {
    public static final int COMPRESSOR_ID = 1;
    public static final int SOLENOID_ID_1 = 2;
    public static final int SOLENOID_ID_2 = 3;

    public static final boolean SOLENOID_CLAW_OPEN_STATE = true;
  }


  public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1);

  public static final class ControllerConversions{
    public static final double DEADBAND = 0.025;
    public static final UnaryOperator<Double> conversionFunction = (input) -> (Math.copySign(input * input, input));
  }

  public static final class SwerveDrivetrain {
    public static final double AUTO_BALANCE_SPEED = 0.35;
    public static final double DOCK_SPEED = 0.8;
    public static final double MIN_BALANCE_TIME = 2;


    public static final Pose3d limelightPose = new Pose3d(0, Units.inchesToMeters(12.25), Units.inchesToMeters(5.75), new Rotation3d(0, 0, toRadians(90)));

    // Distance between centers of right and left wheels on robot in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    // Distance between front and back wheels on robot in meters
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75);

    // Maximum linear chassis speed in meters per second (MK4 standard modules capable of 4.1)
    public static final double MAX_LINEAR_SPEED = 3;
    public static final double MAX_LINEAR_ACCELERATION = 6;

    public static final double MAX_LINEAR_SPEED_AUTO = 1.5;
    public static final double MAX_LINEAR_ACCELERATION_AUTO = 1.5;

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
    public static final double baseToTurret = inchesToMeters(7.5);//Distance from the floor to the top of the turret plate
    public static final double turretToShoulder = inchesToMeters(14); //Distance from the turret to the arm (when the elevator is at 0)
    public static final double shoulderToWrist = inchesToMeters(28.75);
    public static final double wristToEndEffector = inchesToMeters(25.5);

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
    public static final double TURRET_MAX_VEL = 400; //1000
    public static final double TURRET_MAX_ACCEL = 400; //1000
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
    public static final double ELEVATOR_MAX_VEL = 50; //rot/sec 3000
    public static final double ELEVATOR_MAX_ACCEL = 50; //rot/sec^2 5000

    //Shoulder hardware details
    public static final int SHOULDER_ID = 23;
    public static final double SHOULDER_INPUT_TO_OUTPUT = (1.0/70.0) * (2.0 / 3.0) * 2 * PI; //Rotations --> Radians
    public static final int SHOULDER_ENCODER_PORT = 8;
    public static final double SHOULDER_ENCODER_OFFSET = 110.983395; //Degrees
    public static final Range shoulderRange = Range.fromDegrees(-35, 115); //TODO
    public static final double SHOULDER_MAX_VEL = toRadians(40); //Radians/sec
    public static final double SHOULDER_MAX_ACCEL = toRadians(30); //Radians/sec^2

    //Wrist hardware details
    public static final int WRIST_ID = 24;
    public static final double WRIST_INPUT_TO_OUTPUT = (1.0 / 35.0) * 2 * PI; //Ticks --> Radians
    public static final int WRIST_ENCODER_PORT = 9;
    public static final double WRIST_ENCODER_OFFSET = 93.733337; //Degrees
    public static final Range wristRange = Range.fromDegrees(-155, 0); //Degrees //TODO:
    public static final double WRIST_MAX_VEL = toRadians(180); //Radians/sec
    public static final double WRIST_MAX_ACCEL = toRadians(180); //Radians/sec^2

    //Rotator hardware details
    public static final int ROTATOR_ID = 25;
    public static final double ROTATOR_ENCODER_OFFSET = toRadians(48.112586); //Radians //TODO
    public static final Range rotatorRange = Range.fromDegrees(-180, 180);

    //PID gains
    public static final PIDSVGains TURRET_GAINS = new PIDSVGains(10, 0, 0, 0.35, 0.114);
    public static final PIDSVGains ELEVATOR_GAINS = new PIDSVGains(2, 0, 0, 0.3, 0.116);
    public static final PIDSVGains SHOULDER_GAINS = new PIDSVGains(0.35, 0, 0, 0.4, .15);
    public static final PIDGains SHOULDER_CONT_GAINS = new PIDGains(2.5, 0, 0);
    public static final PIDSVGains WRIST_GAINS = new PIDSVGains(.35, 0, 0, 0, 0.14); 
    public static final PIDGains WRIST_CONT_GAINS = new PIDGains(3.5, 0, 0);

    public static final PIDFGains ROTATOR_GAINS = new PIDFGains(2.5, 0, 100, 0);

    //Other constants
    public static final double END_TOLERANCE_CONE_ANGLE = toRadians(2); //Radians
    public static final double ELEVATOR_TOLERANCE = Units.inchesToMeters(16);
    public static final double SHOULDER_ELEVATOR_THRESHOLD = toRadians(75); // The point at which we can start moving the elevator whilst moving the shoulder
    public static final double SHOULDER_REQUIRED_STOWED_HEIGHT = toRadians(30); //The height that the shoulder has to be at before the shoulder doesn't need to move
    
    //Arm setpoints
    public static final ArmState STOWED_POS = new ArmState(0, 0, toRadians(112), toRadians(-150), 0);
    public static final ArmState PICKUP_DOUBLE_POS = new ArmState(0, 0, toRadians(98.7), toRadians(-115), 0);
    public static final ArmState GROUND_PICKUP_POS = new ArmState(0, Units.inchesToMeters(0), toRadians(16), toRadians(-81), 0);
    public static final ArmState HIGH_POS = new ArmState(0, Units.inchesToMeters(22), toRadians(17), toRadians(4), 0);
    public static final ArmState MID_POS = new ArmState(0, 0, toRadians(65), toRadians(-75), 0);
    public static final ArmState LOW_POS = new ArmState(0, 0, toRadians(71), toRadians(-139), 0);
    public static final ArmState HIGH_CUBE_POS = new ArmState(0, 0, toRadians(49), toRadians(-32), 0);
  }

  public static class Lights {
    public static final int CANDLE_ID = 30;
    public static final double brightness = 1;
    public static final int NUM_LEDS = 308;

    public static final double BOUNCE_SPEED = 0.75;
    public static final double BLINK_SPEED = 0.5;

    public static final Animation DISABLED_ANIMATION =
            new LarsonAnimation(0, 0, 255, 0, BOUNCE_SPEED, NUM_LEDS, Front, 7);

    public static final RGB IDLE_RGB = new RGB(0, 0, 255);

    public static final RGB ELEMENT_GRABBED_RGB = new RGB(0, 255, 0);
    public static final Animation DEPLOYING_ANIMATION =
            new StrobeAnimation(0, 0, 255, 0, BLINK_SPEED, NUM_LEDS);

    public static final RGB SCORING_RGB = new RGB(0, 255, 0);


    public static final RGB UPRIGHT_CONE_RGB = new RGB(255, 255, 0);

    public static final Animation DOWNED_CONE_ANIMATION =
            new StrobeAnimation(255, 255, 0, 0, BLINK_SPEED, NUM_LEDS);

    public static final RGB CUBE_RGB = new RGB(144,22,153);
  }

  public static void pullAllianceFromFMS(RobotContainer rc) {
    boolean isRedAlliance = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    if(!overrideAlliance) alliance = isRedAlliance ? Alliance.Red : Alliance.Blue;

    rc.arm().reInstantiateGridUI(alliance);
  }
  
}
