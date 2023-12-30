package frc.robot.subsystems.claw;

import frc.robot.subsystems.claw.Claw.ClawState;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {

  @AutoLog
  public static class ClawIOInputs {
    public double test = 0.0;
    public ClawState desiredState;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}
}
