package frc.robot.subsystems.arm;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Turret.*;
import static frc.robot.Constants.applyCurrentLimit;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;
import frc.robot.commands.WhileDisabledInstantCommand;

public class ArmIOReal implements ArmIO {

  protected final MotionMagicTalonFX elevator =
      new MotionMagicTalonFX(
          ELEVATOR_ID,
          ELEVATOR_GAINS,
          ELEVATOR_INPUT_TO_OUTPUT,
          ELEVATOR_MAX_VEL,
          ELEVATOR_MAX_ACCEL,
          2500);

  protected final MotionMagicTalonFX shoulderLeader =
      new MotionMagicTalonFX(
          SHOULDER_LEADER_ID,
          SHOULDER_GAINS,
          SHOULDER_INPUT_TO_OUTPUT,
          SHOULDER_VEL,
          SHOULDER_ACCEL,
          10000);

  protected final MotionMagicTalonFX shoulderFollower =
      new MotionMagicTalonFX(
          SHOULDER_FOLLOWER_ID,
          SHOULDER_GAINS,
          SHOULDER_INPUT_TO_OUTPUT,
          SHOULDER_VEL,
          SHOULDER_ACCEL,
          SHOULDER_JERK);

  protected final ThroughBoreEncoder shoulderEncoder =
      new ThroughBoreEncoder(SHOULDER_ENCODER_PORT, SHOULDER_ENCODER_OFFSET);

  // Wrist hardware
  protected final MotionMagicTalonFX wrist =
      new MotionMagicTalonFX(
          WRIST_ID, WRIST_GAINS, WRIST_INPUT_TO_OUTPUT, WRIST_VEL, WRIST_ACCEL, WRIST_JERK);

  protected final ThroughBoreEncoder wristEncoder =
      new ThroughBoreEncoder(WRIST_ENCODER_PORT, WRIST_ENCODER_OFFSET);

  public ArmIOReal(boolean isSim) {

    configureHardware();
    if (!isSim) {
      configureCurrentLimits();
    }
  }

  public ArmIOReal() {
    this(false);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.elevatorTarget = elevator.getTarget();
    inputs.elevatorPos = elevator.getEncoderPosition();

    inputs.shoulderTarget = shoulderLeader.getTarget();
    inputs.shoulderAbsolutePos = shoulderEncoder.getRadians();
    inputs.shoulderMotorPos = shoulderLeader.getEncoderPosition();

    inputs.wristTarget = wrist.getTarget();
    inputs.wristAbsolutePos = wristEncoder.getRadians();
    inputs.wristMotorPos = wrist.getEncoderPosition();
  }

  protected void configureHardware() {
    shoulderEncoder.setInverted(false);
    wristEncoder.setInverted(true);

    elevator.configure(Brake, CounterClockwise_Positive);

    shoulderLeader.configure(Brake, Clockwise_Positive);

    shoulderFollower.configure(Brake, Clockwise_Positive);

    wrist.configure(Brake, CounterClockwise_Positive);
  }

  private void configureCurrentLimits() {
    applyCurrentLimit(elevator);
    applyCurrentLimit(shoulderLeader);
    applyCurrentLimit(shoulderFollower);
    applyCurrentLimit(wrist);
  }

  @Override
  public Command setShoulderFollower() {
    return new WhileDisabledInstantCommand(
        () -> shoulderFollower.setControl(new Follower(shoulderLeader.getDeviceID(), false)));
  }

  @Override
  public void stop() {
    elevator.set(0);
    shoulderLeader.set(0);
    wrist.set(0);
  }

  @Override
  public void setElevatorTarget(double target) {
    elevator.setTarget(target);
  }

  @Override
  public void setShoulderTarget(double target) {
    shoulderLeader.setTarget(target);
  }

  @Override
  public void setWristTarget(double target) {
    wrist.setTarget(target);
  }

  @Override
  public void resetShoulderPos(double pos) {
    shoulderLeader.resetPosition(pos);
  }

  @Override
  public void resetWristPos(double pos) {
    wrist.resetPosition(pos);
  }

  @Override
  public void changeShoulderSpeed(double vel, double accel, double jerk) {
    shoulderLeader.changeSpeed(vel, accel, jerk);
  }

  @Override
  public void changeWristSpeed(double vel, double accel, double jerk) {
    wrist.changeSpeed(vel, accel, jerk);
  }
}
