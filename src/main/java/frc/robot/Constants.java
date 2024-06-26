// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static com.ctre.phoenix.led.LarsonAnimation.BounceMode.Front;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleSpeedLevel.L2;
import static frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleType.MK4i;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.Candle.MultipleColorSegments;
import frc.robot.ShamLib.Candle.RGB;
import frc.robot.ShamLib.Candle.RGBSegmentInfo;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.swerve.SwerveDriveConfig;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import frc.robot.ShamLib.swerve.odometry.OdometryBoundingBox;
import frc.robot.subsystems.BaseVision.CamSettings;
import frc.robot.util.LUT;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.math.Range;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

public final class Constants {

  public static final double loopPeriodSecs = 0.02;
  public static BuildMode currentBuildMode = BuildMode.SIM;

  public static boolean AT_COMP = false;

  public static final class PhysicalConstants {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    public static final OdometryBoundingBox FIELD_BOUNDING_BOX =
        new OdometryBoundingBox(
            new Translation2d(), new Translation2d(Units.feetToMeters(54), Units.feetToMeters(27)));

    static {
      try {
        APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      } catch (Exception e) {
        throw new RuntimeException("Could not load AprilTag field layout from WPI");
      }
    }
  }

  public static final class Testing {

    public static final Trigger RAISE = new Trigger(() -> false);
    public static final Trigger LOWER = new Trigger(() -> false);
    public static final Trigger STOP = new Trigger(() -> true);
  }

  public static boolean HAS_BEEN_ENABLED = false;
  public static boolean TURRET_ZEROED = false;
  public static final double VOLTAGE_WARNING = 9;
  public static Alliance alliance = Alliance.Red;
  public static boolean overrideAlliance =
      false; // Flag to indicate that the drivers have manually set the allianc
  // public static final GridInterface gridInterface = new GridInterface(alliance);
  public static boolean gridReinstantiated = true;

  public static final class Claw {

    public static final int COMPRESSOR_ID = 1;
    public static final int SOLENOID_ID_1 = 2;
    public static final int SOLENOID_ID_2 = 3;

    public static final DoubleSolenoid.Value SOLENOID_CLAW_OPEN_VALUE =
        DoubleSolenoid.Value.kReverse;

    public static final int PROX_PORT = 9;
  }

  public static final CurrentLimitsConfigs CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  public static final class ControllerConversions {

    public static final double DEADBAND = 0.025;
    public static final UnaryOperator<Double> conversionFunction =
        (input) -> (Math.copySign(input * input, input));
  }

  public static final class SwerveDrivetrain {

    public static final class AutoBalance {

      public static final PIDGains AUTO_BALANCE_GAINS = new PIDGains(0.024, 0, 0.011);

      public static final double DOCK_THRESHOLD = 15;

      public static final double NO_ANGLE_CHECK_TIME = 2;

      public static final double AUTO_BALANCE_SPEED = 0.6;
      public static final double DOCK_SPEED = 1.2;
      public static final double DRIVE_OVER_SPEED = 0.6;

      // in 1/50s of a second how long the bot should be balanced for before the autobalance command
      // exits
      public static final int AUTO_BALANCE_BUFFER_SIZE = 13;

      public static final int DRIVE_OVER_BUFFER_SIZE = 40; // 40
    }

    // Distance between centers of right and left wheels on robot in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    // Distance between front and back wheels on robot in meters
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75);

    public static final double rotationRadius =
        Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * PI;

    // Standard speeds (MK4 standard modules capable of 4.9)
    public static final double STANDARD_LINEAR_SPEED = 3;
    public static final double STANDARD_LINEAR_ACCELERATION = 6;
    public static final double STANDARD_ROTATION =
        (STANDARD_LINEAR_SPEED / rotationRadius) * (2 * PI);
    public static final double STANDARD_ROT_ACCEL = STANDARD_ROTATION * 3;

    public static final SwerveSpeedLimits STANDARD_SPEED =
        new SwerveSpeedLimits(
            STANDARD_LINEAR_SPEED,
            STANDARD_LINEAR_ACCELERATION,
            STANDARD_ROTATION,
            STANDARD_ROT_ACCEL);

    // Max speeds (turbo button)
    public static final double MAX_LINEAR_SPEED = 5;
    public static final double MAX_LINEAR_ACCELERATION = 7.5;
    public static final double MAX_ROTATION = (MAX_LINEAR_SPEED / rotationRadius) * (2 * PI);
    public static final double MAX_ROT_ACCEL = MAX_ROTATION * 3;

    public static final SwerveSpeedLimits MAX_SPEED =
        new SwerveSpeedLimits(
            MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_ROTATION, MAX_ROT_ACCEL);

    // Min speed to allow turbo button to work
    public static final double MIN_TURBO_SPEED = 2.5;

    public static final double MAX_LINEAR_SPEED_AUTO = 1.5;
    public static final double MAX_LINEAR_ACCELERATION_AUTO = 1.5;

    public static final Translation2d[] moduleOffsets = {
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // front left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // back left
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // back right
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2) // front right
    };

    public static final Matrix<N3, N1> STATE_STD_DEVIATIONS = VecBuilder.fill(0.003, 0.003, 0.0002);

    public static final double P_HOLDANGLETELE = 0.25;
    public static final double I_HOLDANGLETELE = 0;
    public static final double D_HOLDANGLETELE = 0;

    public static final PIDGains HOLD_ANGLE_GAINS_TELE =
        new PIDGains(P_HOLDANGLETELE, I_HOLDANGLETELE, D_HOLDANGLETELE);

    public static final double P_HOLDANGLEAUTO = 5;
    public static final double I_HOLDANGLEAUTO = 0;
    public static final double D_HOLDANGLEAUTO = 0;

    public static final PIDGains HOLD_ANGLE_GAINS_AUTO =
        new PIDGains(P_HOLDANGLEAUTO, I_HOLDANGLEAUTO, D_HOLDANGLEAUTO);

    public static final double P_HOLDTRANSLATION = 3;
    public static final double I_HOLDTRANSLATION = 0;
    public static final double D_HOLDTRANSLATION = 0;

    public static final PIDGains HOLD_TRANSLATION_GAINS =
        new PIDGains(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION);

    public static final SwerveModuleState STOPPED_STATE =
        new SwerveModuleState(0, new Rotation2d());

    public static final SwerveModuleState[] X_SHAPE_ARRAY = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
    };

    public static final int PIGEON_ID = 01;

    // front left
    public static final int MODULE_1_DRIVE_ID = 11;
    public static final int MODULE_1_TURN_ID = 12;
    public static final int MODULE_1_ENCODER_ID = 11;
    public static final double MODULE_1_OFFSET = -119.002734;

    // back left
    public static final int MODULE_2_DRIVE_ID = 13;
    public static final int MODULE_2_TURN_ID = 14;
    public static final int MODULE_2_ENCODER_ID = 13;
    public static final double MODULE_2_OFFSET = -15.553125;

    // back right
    public static final int MODULE_3_DRIVE_ID = 15;
    public static final int MODULE_3_TURN_ID = 16;
    public static final int MODULE_3_ENCODER_ID = 15;
    public static final double MODULE_3_OFFSET = -113.199023;

    // front right
    public static final int MODULE_4_DRIVE_ID = 17;
    public static final int MODULE_4_TURN_ID = 18;
    public static final int MODULE_4_ENCODER_ID = 17;
    public static final double MODULE_4_OFFSET = 30.585938;

    public static ModuleInfo MODULE_1_INFO =
        ModuleInfo.generateModuleInfo(
            MK4i,
            L2,
            MODULE_1_DRIVE_ID,
            MODULE_1_TURN_ID,
            MODULE_1_ENCODER_ID,
            MODULE_1_OFFSET,
            moduleOffsets[0],
            false,
            false,
            false);

    public static ModuleInfo MODULE_2_INFO =
        ModuleInfo.generateModuleInfo(
            MK4i,
            L2,
            MODULE_2_DRIVE_ID,
            MODULE_2_TURN_ID,
            MODULE_2_ENCODER_ID,
            MODULE_2_OFFSET,
            moduleOffsets[1],
            false,
            true,
            true);

    public static ModuleInfo MODULE_3_INFO =
        ModuleInfo.generateModuleInfo(
            MK4i,
            L2,
            MODULE_3_DRIVE_ID,
            MODULE_3_TURN_ID,
            MODULE_3_ENCODER_ID,
            MODULE_3_OFFSET,
            moduleOffsets[2],
            false,
            true,
            true);

    public static ModuleInfo MODULE_4_INFO =
        ModuleInfo.generateModuleInfo(
            MK4i,
            L2,
            MODULE_4_DRIVE_ID,
            MODULE_4_TURN_ID,
            MODULE_4_ENCODER_ID,
            MODULE_4_OFFSET,
            moduleOffsets[3],
            false,
            true,
            true);

    public static Supplier<Pose2d> getOdoPose;
    public static Supplier<Rotation2d> getDrivetrainAngle;

    public static final SwerveDriveConfig SWERVE_CONFIG = new SwerveDriveConfig();

    static {
      SWERVE_CONFIG.setModuleInfos(MODULE_1_INFO, MODULE_2_INFO, MODULE_3_INFO, MODULE_4_INFO);
      SWERVE_CONFIG.pigeon2ID = PIGEON_ID;
      SWERVE_CONFIG.gyroCanbus = "";

      SWERVE_CONFIG.moduleDriveGains = SwerveModule.DRIVE_GAINS;
      SWERVE_CONFIG.moduleTurnGains = SwerveModule.TURN_GAINS;

      SWERVE_CONFIG.maxModuleTurnVelo = SwerveModule.MAX_TURN_SPEED;
      SWERVE_CONFIG.maxModuleTurnAccel = SwerveModule.MAX_TURN_ACCEL;

      SWERVE_CONFIG.moduleCanbus = "drivetrain";

      SWERVE_CONFIG.standardSpeedLimits = STANDARD_SPEED;

      SWERVE_CONFIG.autoThetaGains = HOLD_ANGLE_GAINS_AUTO;
      SWERVE_CONFIG.translationGains = HOLD_TRANSLATION_GAINS;
      SWERVE_CONFIG.currentLimit = CURRENT_LIMIT;

      SWERVE_CONFIG.standardDeviations = null;
      SWERVE_CONFIG.loopPeriod = loopPeriodSecs;
    }
  }

  public static final class SwerveModule {

    public static final double MAX_TURN_SPEED = 1000; // motor rots / sec
    public static final double MAX_TURN_ACCEL = 1000; // motor rots / sec^2

    // Turn motor coefficients
    public static final PIDSVGains TURN_GAINS =
        new PIDSVGains(
            10, // 10
            0,
            0,
            0.3, // 0.3
            0.121057 // 0.112
            );

    // Drive motor coefficients
    public static final PIDSVGains DRIVE_GAINS = new PIDSVGains(0.25, 0, 0, 0.3, 0.1135);
  }

  public static final class Vision {

    public static final Pose3d BASE_LIMELIGHT_POSE =
        new Pose3d(inchesToMeters(-7.549165), 0, inchesToMeters(8.585326 + 1.44), new Rotation3d());

    // Base
    public static final int APRIL_TAG_PIPELINE = 0;
    public static final int RETRO_PIPELINE = 1;

    public static final CamSettings BASE_LIMELIGHT_SETTINGS =
        new CamSettings("base_ll", new Pose3d(), Units.feetToMeters(18), 0.4, 2.0, 0.33, 1.0);

    public static DoubleSupplier BASE_X_OFFSET_SUPPLIER = () -> 0;
    public static BooleanSupplier BASE_HAS_TARGET_SUPPLIER = () -> false;

    // Claw
    public static final int CONE_DETECTOR_PIPELINE = 0;
    public static final int CUBE_DETECTOR_PIPELINE = 1;
    public static final int ELEMENT_TYPE_PIPELINE = 2;
  }

  public static class Arm {
    // Physial constants for arm dimensions

    public static final double baseToTurret =
        inchesToMeters(7.5); // Distance from the floor to the top of the turret plate
    public static final double turretToShoulder =
        inchesToMeters(14); // Distance from the turret to the arm (when the elevator is at 0)
    public static final double shoulderToWrist = inchesToMeters(28.75);
    public static final double wristToEndEffector = inchesToMeters(25.5);

    // Elevator hardware details
    public static final int ELEVATOR_ID = 22;
    public static final double ELEVATOR_INPUT_TO_OUTPUT =
        (1.0 / 15.0)
            * // Gearbox
            1.273
            * PI
            * // Pitch diameter (1.273") --> distance traveled by chain (in inches)
            0.0254 // inches to meters
        ; // Converts motor revolutions to meters
    public static final Range elevatorRange = new Range(0, inchesToMeters(26));
    public static final double ELEVATOR_MAX_VEL = 400; // rot/sec 3000
    public static final double ELEVATOR_MAX_ACCEL = 300; // rot/sec^2 5000

    // Shoulder hardware details
    public static final int SHOULDER_LEADER_ID = 23;
    public static final int SHOULDER_FOLLOWER_ID = 29;
    public static final double SHOULDER_INPUT_TO_OUTPUT =
        (10.0 / 64.0) * (16.0 / 64.0) * (32.0 / 64.0) * 2 * PI; // Rotations --> Radians
    public static final int SHOULDER_ENCODER_PORT = 8;
    public static final double SHOULDER_ENCODER_OFFSET = -57.7; // Degrees
    public static final Range shoulderRange = Range.fromDegrees(-45, 115);

    public static final double SHOULDER_VEL = 25; // 25
    public static final double SHOULDER_ACCEL = 25; // 25
    public static final double SHOULDER_JERK = 1800;

    public static final double SHOULDER_SLOW_VEL = 17; // 50
    public static final double SHOULDER_SLOW_ACCEL = 17; // 100
    public static final double SHOULDER_SLOW_JERK = 1800;

    //    public static final double SHOULDER_SLOW_VEL = toRadians(40); //Radians/sec
    //    public static final double SHOULDER_SLOW_ACCEL = toRadians(80); //Radians/sec^2
    public static final double SHOULDER_FAST_VEL = 25; // 50
    public static final double SHOULDER_FAST_ACCEL = 25; // 100
    public static final double SHOULDER_FAST_JERK = 1800;

    //    public static final double SHOULDER_FAST_VEL = toRadians(400); //Radians/sec
    //    public static final double SHOULDER_FAST_ACCEL = toRadians(500); //Radians/sec^2
    // Wrist hardware details
    public static final int WRIST_ID = 24;
    public static final double WRIST_INPUT_TO_OUTPUT =
        (1.0 / 10.0) * (25.0 / 75.0) * 2 * PI; // Rotations --> Radians
    public static final int WRIST_ENCODER_PORT = 7;
    public static final double WRIST_ENCODER_OFFSET = 33; // Degrees
    public static final Range wristRange = Range.fromDegrees(-155, 45); // Degrees

    public static final double WRIST_VEL = 80; // 80
    public static final double WRIST_ACCEL = 40; // 40
    public static final double WRIST_JERK = 2500;

    public static final double WRIST_SLOW_VEL = 40;
    public static final double WRIST_SLOW_ACCEL = 20;
    public static final double WRIST_SLOW_JERK = 2500;

    // Control gains
    public static final PIDSVGains ELEVATOR_GAINS = new PIDSVGains(2, 0, 0, 0.25, 0.142);

    public static final PIDSVGains SHOULDER_GAINS = new PIDSVGains(2, 0, 0, 0.25, 0.15);

    public static final PIDSVGains WRIST_GAINS = new PIDSVGains(14, 0, 0, 0.3, 0.302927);

    //    public static final PIDGains WRIST_GAINS = new PIDGains(4.5, 0, 0.25);
    //    public static final double WRIST_KS = 0.25;
    //    public static final double WRIST_KG = 0.3;
    //    public static final double WRIST_KV = 0.9; //1.4
    //    public static final PIDFGains ROTATOR_GAINS = new PIDFGains(2.5, 0, 100, 0);
    // Other constants
    public static final double END_TOLERANCE_CONE_ANGLE = toRadians(2); // Radians
    public static final double ELEVATOR_TOLERANCE = Units.inchesToMeters(22);
    public static final double SHOULDER_ELEVATOR_THRESHOLD =
        toRadians(
            75); // The point at which we can start moving the elevator whilst moving the shoulder
    public static final double SHOULDER_REQUIRED_STOWED_HEIGHT =
        toRadians(
            30); // The height that the shoulder has to be at before the shoulder doesn't need to
    // move

    // Arm setpoints
    public static final ArmState STOWED_POS =
        new ArmState(0, 0, toRadians(113), toRadians(-137), 0);
    public static final ArmState PICKUP_DOUBLE_POS =
        new ArmState(0, 0, toRadians(102), toRadians(-114), 0);
    public static final ArmState GROUND_PICKUP_POS =
        new ArmState(0, Units.inchesToMeters(7), toRadians(-25), toRadians(46), 0);
    public static final ArmState HIGH_POS =
        new ArmState(0, Units.inchesToMeters(10.5), toRadians(22), toRadians(17), 0);
    public static final ArmState MID_POS = new ArmState(0, 0, toRadians(68), toRadians(-73), 0);
    public static final ArmState LOW_POS = new ArmState(0, 0, toRadians(71), toRadians(-139), 0);
    public static final ArmState NEW_GROUND_PICKUP_POS =
        new ArmState(0, toRadians(25), toRadians(-108));
    public static final ArmState NEW_INTERMEDIATE_GROUND_PICKUP_POS =
        new ArmState(0, toRadians(35), toRadians(-108));
    public static final ArmState TELEOP_GROUND_INTERMEDIATE_POS =
        new ArmState(0, toRadians(45), toRadians(-108));
    public static final ArmState HIGH_CUBE_POS =
        new ArmState(0, 0, toRadians(40), toRadians(-12), 0);
    public static final ArmState PRIMED_POS = new ArmState(0, 0, toRadians(90), toRadians(-90), 0);
  }

  public static class Turret {
    // Turret hardawre details

    public static final int TURRET_ID = 21;
    public static final double TURRET_INPUT_TO_OUTPUT =
        (1.0 / 15.0) * (10.0 / 140.0) * 2 * PI // To radians
        ; // Rotations --> Radians
    public static final int TURRET_POT_PORT = 0; // Analog port
    public static final double TURRET_POT_RATIO =
        -514.2857142857143; // Converts turns of the potentiometer to output degrees
    public static final double TURRET_ENCODER_OFFSET = 257; // Degrees
    public static final double TURRET_MAX_VEL = 400; // 1000
    public static final double TURRET_MAX_ACCEL = 300; // 1000
    public static final double TURRET_SLOW_VEL = 100;
    public static final double TURRET_SLOW_ACCEL = 100;
    public static final Range TURRET_RANGE = Range.fromDegrees(-180, 180);

    public static final double TURRET_START_ANGLE = toRadians(-90);

    // public static final PIDSVGains TURRET_GAINS = new PIDSVGains(10, 0, 0, 0.35, 0.114);
    public static final PIDSVGains TURRET_GAINS = new PIDSVGains(10, 0, 0, 0.5, 0.113);

    public static final double MANUAL_CONTROL_VELOCITY = 15; // Deg / sec
    public static final double MANUAL_CONTROL_BUMP = toRadians(2);

    public static final double TURRET_ALLOWED_ERROR = toRadians(2);

    public static final LUT<Double, Double> AIMING_LUT =
        new LUT<Double, Double>() {
          {
            add(toRadians(30.0), .2);
            add(toRadians(25.0), .2);
            add(toRadians(20.0), .2);
            add(toRadians(15.0), .2);
            add(toRadians(10.0), .4);
            add(toRadians(5.0), .6);
            add(toRadians(0.0), 1.0);
          }
        };

    public static double getTurretStartAngleFromAlliance() {
      return (Constants.alliance == Alliance.Red ? 1 : -1) * Constants.Turret.TURRET_START_ANGLE;
    }
  }

  public static class Lights {

    public static final int CANDLE_ID = 30;
    public static final double brightness = 1;
    public static final int NUM_LIGHTS = 305;
    public static final int NUM_LIGHTS_WITHOUT_CANDLE = NUM_LIGHTS - 8;

    public static final double BOUNCE_SPEED = 0.75;
    public static final double BLINK_SPEED = .075;

    public static final Animation DISABLED_ANIMATION =
        new LarsonAnimation(0, 0, 255, 0, BOUNCE_SPEED, NUM_LIGHTS_WITHOUT_CANDLE, Front, 7, 8);

    public static final RGB IDLE_RGB = new RGB(0, 0, 255);

    public static final RGB ELEMENT_GRABBED_RGB = new RGB(0, 255, 0);
    public static final Animation SCORING_ANIMATION =
        new StrobeAnimation(0, 0, 255, 0, BLINK_SPEED, NUM_LIGHTS);

    public static final RGB CONE_RGB = new RGB(255, 255, 0);

    public static final Animation INTAKE_CONE_ANIMATION =
        new StrobeAnimation(255, 255, 0, 0, BLINK_SPEED, NUM_LIGHTS);

    public static final RGB CUBE_RGB = new RGB(144, 22, 153);

    public static final Animation INTAKE_CUBE_ANIMATION =
        new StrobeAnimation(144, 22, 153, 0, BLINK_SPEED, NUM_LIGHTS);

    public static final MultipleColorSegments HAVE_CONE_WANT_CUBE =
        new MultipleColorSegments(
            8,
            new RGBSegmentInfo(CUBE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4),
            new RGBSegmentInfo(CONE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4),
            new RGBSegmentInfo(CUBE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4),
            new RGBSegmentInfo(CONE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4));

    public static final MultipleColorSegments HAVE_CUBE_WANT_CONE =
        new MultipleColorSegments(
            8,
            new RGBSegmentInfo(CONE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4),
            new RGBSegmentInfo(CUBE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4),
            new RGBSegmentInfo(CONE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4),
            new RGBSegmentInfo(CUBE_RGB, NUM_LIGHTS_WITHOUT_CANDLE / 4));

    public static final Animation SOFT_STOP_ANIMATION =
        new StrobeAnimation(255, 0, 0, 0, BLINK_SPEED, NUM_LIGHTS);

    public static final RGB ENABLE_PROX_RGB = new RGB(0, 255, 0);
    public static final RGB DISABLE_PROX_RGB = new RGB(255, 0, 0);

    //    public static final Animation AUTO_ANIMATION = new TwinkleAnimation(0, 0, 255, 0, .95,
    // NUM_LIGHTS, TwinkleAnimation.TwinklePercent.Percent30);
    public static final Animation AUTO_ANIMATION =
        new ColorFlowAnimation(
            0, 0, 255, 0, .0125, NUM_LIGHTS, ColorFlowAnimation.Direction.Forward);
  }

  public static void pullAllianceFromFMS(RobotContainer rc) {
    boolean isRedAlliance =
        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getEntry("IsRedAlliance")
            .getBoolean(true);
    if (!overrideAlliance) {
      alliance = isRedAlliance ? Alliance.Red : Alliance.Blue;
    }

    // reInstantiateGridUI(alliance);
  }

  public static CurrentLimitsConfigs getCurrentLimit() {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    configs.SupplyCurrentLimitEnable = true;
    configs.SupplyCurrentLimit = 20;
    return configs;
  }

  public static void applyCurrentLimit(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.CurrentLimits = getCurrentLimit();
    motor.getConfigurator().apply(config);
  }

  public static enum ElementType {
    Cube,
    Cone,
    None
  }

  // public static void reInstantiateGridUI(Alliance alliance) {
  // gridInterface.setAlliance(alliance);
  // Constants.gridReinstantiated = true;
  // }
}
