package frc.robot.subsystems.turret;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;

public class TurretIOReal implements TurretIO {

  MotionMagicTalonFX turretMotor =
      new MotionMagicTalonFX(
          TURRET_ID, TURRET_GAINS, TURRET_INPUT_TO_OUTPUT, TURRET_MAX_VEL, TURRET_MAX_ACCEL, 2500);

  private final AnalogPotentiometer turretPotentiometer =
      new AnalogPotentiometer(TURRET_POT_PORT, TURRET_POT_RATIO, TURRET_ENCODER_OFFSET);

  public TurretIOReal() {
    turretMotor.configure(NeutralModeValue.Coast, InvertedValue.Clockwise_Positive);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.potentiometerAngle = Math.toRadians(turretPotentiometer.get());
    inputs.turretMotorAngle = turretMotor.getEncoderPosition();
    inputs.turretTarget = turretMotor.getTarget();
  }

  @Override
  public void changeSpeed(double vel, double accel, double jerk) {
    turretMotor.changeSpeed(vel, accel, jerk);
  }

  @Override
  public void setPower(double power) {
    turretMotor.set(power);
  }

  @Override
  public void resetMotorAngle(double value) {
    turretMotor.resetPosition(value);
  }

  @Override
  public void setTarget(double angle) {
    turretMotor.setTarget(angle);
  }
}
