package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double potentiometerAngle = 0.0; // rads
    public double turretTarget = 0.0; // rads
    public double turretMotorAngle = 0.0; // rads
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void changeSpeed(double vel, double accel, double jerk) {}

  public default void setPower(double power) {}

  public default void resetMotorAngle(double value) {}

  public default void setTarget(double angle) {}

  public default Command calculateFF(Trigger increment, BooleanSupplier interrupt) {
    return new InstantCommand();
  }
}
