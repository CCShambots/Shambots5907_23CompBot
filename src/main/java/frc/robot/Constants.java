// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.*;
import frc.robot.ShamLib.Candle.RGB;

import static com.ctre.phoenix.led.LarsonAnimation.BounceMode.Front;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

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

  public static final class Vision {
    public static Pose3d baseLimelightPose = new Pose3d(inchesToMeters(-5), 0, inchesToMeters(31), new Rotation3d());

    //Base
    public static final int APRIL_TAG_PIPELINE = 0;

    //Claw
    public static final int CONE_DETECTOR_PIPELINE = 0;
    public static final int CUBE_DETECTOR_PIPELINE = 1;
    public static final int CONE_ORIENTATION_PIPELINE = 2;
  }

  public static class Lights {
    public static final int CANDLE_ID = 30;
    public static final double brightness = 1;
    public static final int NUM_LEDS = 50;

    public static final double BOUNCE_SPEED = 0.5;
    public static final double BLINK_SPEED = 0.5;

    public static final Animation DISABLED_ANIMATION =
            new LarsonAnimation(0, 0, 255, 0, BOUNCE_SPEED, NUM_LEDS, Front, 5);

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
}
