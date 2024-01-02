package frc.robot.subsystems.claw;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {

  @AutoLog
  public static class ClawIOInputs {
    public DoubleSolenoid.Value solenoidValue = kOff;
    public boolean proxActivated = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  public default void open() {}

  public default void close() {}
}
