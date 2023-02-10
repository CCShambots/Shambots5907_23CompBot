// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.*;
import frc.robot.ShamLib.Candle.RGB;

import static com.ctre.phoenix.led.LarsonAnimation.BounceMode.Front;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Lights {
    public static final int CANDLE_ID = 0;
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


    public static final RGB UPRIGHT_CONE = new RGB(255, 255, 0);

    public static final Animation DOWNED_CONE_ANIMATION =
            new StrobeAnimation(255, 255, 0, 0, BLINK_SPEED, NUM_LEDS);

    public static final RGB CUBE_RGB = new RGB(144,22,153);
  }
}
