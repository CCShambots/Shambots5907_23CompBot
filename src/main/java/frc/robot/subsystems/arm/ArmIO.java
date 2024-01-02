package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double elevatorTarget = 0.0; // meters
    public double elevatorPos = 0.0; // meters

    public double shoulderTarget = 0.0; // rads
    public double shoulderAbsolutePos = 0.0; // rads
    public double shoulderMotorPos = 0.0; // rads

    public double wristTarget = 0.0; // rads
    public double wristAbsolutePos = 0.0; // rads
    public double wristMotorPos = 0.0; // rads
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default Command setShoulderFollower() {
    return new InstantCommand();
  }

  public default Command calculateElevatorFF(Trigger increment, BooleanSupplier interrupt) {
    return new InstantCommand();
  }

  public default Command calculateShoulderFF(Trigger increment, BooleanSupplier interrupt) {
    return new InstantCommand();
  }

  public default Command calculateWristFF(Trigger increment, BooleanSupplier interrupt) {
    return new InstantCommand();
  }

  public default void stop() {}

  public default void setElevatorTarget(double target) {}

  public default void setShoulderTarget(double target) {}

  public default void setWristTarget(double target) {}

  // public default void resetElevatorPos(double target) {}
  public default void resetShoulderPos(double pos) {}

  public default void resetWristPos(double pos) {}

  public default void changeShoulderSpeed(double vel, double accel, double jerk) {}

  public default void changeWristSpeed(double vel, double accel, double jerk) {}
}
