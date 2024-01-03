package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.subsystems.arm.Arm.ArmMode.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawState;
import frc.robot.subsystems.claw.ClawIOReal;
import frc.robot.util.kinematics.ArmKinematics;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.kinematics.ArmTrajectory;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends StateMachine<Arm.ArmMode> {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final ArmKinematics kinematics =
      new ArmKinematics(baseToTurret, turretToShoulder, shoulderToWrist, wristToEndEffector);

  private final Claw claw = new Claw(new ClawIOReal());

  private ArmState currentArmState = STOWED_POS;

  public Arm(ArmIO io) {
    super("Arm", UNDETERMINED, ArmMode.class);

    this.io = io;

    io.updateInputs(inputs);

    new WaitCommand(2).andThen(setShoulderFollower()).schedule();

    addChildSubsystem(claw);

    defineTransitions();
    registerStateCommands();

    // TODO: uncomment
    // goToArmState(STOWED_POS);
  }

  public Command setShoulderFollower() {
    return io.setShoulderFollower();
  }

  public Command openClaw() {
    return new InstantCommand(
        () -> {
          claw.requestTransition(ClawState.OPENED);
        });
  }

  public Command closeClaw() {
    return new InstantCommand(
        () -> {
          claw.requestTransition(ClawState.CLOSED);
        });
  }

  public ClawState getClawState() {
    return claw.getState();
  }

  public Command enableClawProx() {
    return new InstantCommand(claw::enableProx);
  }

  public Command disableClawProx() {
    return new InstantCommand(claw::disableProx);
  }

  private void defineTransitions() {
    addOmniTransition(SEEKING_STOWED);
    addTransition(SEEKING_STOWED, STOWED);

    addOmniTransition(
        SOFT_STOP,
        () -> {
          io.stop();
        });

    addTransition(STOWED, PRIMED, () -> goToArmState(PRIMED_POS));
    addOmniTransition(SEEKING_PRIMED);
    addTransition(SEEKING_PRIMED, PRIMED);
    addOmniTransition(SEEKING_PICKUP_GROUND);

    addTransition(
        PRIMED, NEW_GROUND_INTERMEDIATE, () -> goToArmState(NEW_INTERMEDIATE_GROUND_PICKUP_POS));
    addTransition(
        NEW_GROUND_INTERMEDIATE, NEW_GROUND_PICKUP, () -> goToArmState(NEW_GROUND_PICKUP_POS));
    addTransition(NEW_GROUND_PICKUP, LOW_SCORE, () -> goToArmState(LOW_POS));

    // New ground pickup stuff
    // Make sure we can go to and from low score (for auto)
    addTransition(LOW_SCORE, NEW_GROUND_PICKUP, () -> goToArmState(NEW_GROUND_PICKUP_POS));
    addTransition(
        LOW_SCORE, NEW_GROUND_INTERMEDIATE, () -> goToArmState(NEW_INTERMEDIATE_GROUND_PICKUP_POS));

    // Teleop stuff
    addTransition(
        STOWED,
        NEW_GROUND_PICKUP,
        () -> {
          goToArmState(NEW_GROUND_PICKUP_POS);
        });
    addTransition(
        STOWED,
        TELEOP_GROUND_INTERMEDIATE,
        () -> {
          goToArmState(TELEOP_GROUND_INTERMEDIATE_POS);
          claw.disableProx();
        });

    addTransition(
        TELEOP_GROUND_INTERMEDIATE, NEW_GROUND_PICKUP, () -> goToArmState(NEW_GROUND_PICKUP_POS));

    // Make sure we can go from pickup to intermediate (for screwing around in the community)
    addTransition(
        NEW_GROUND_PICKUP,
        TELEOP_GROUND_INTERMEDIATE,
        () -> goToArmState(TELEOP_GROUND_INTERMEDIATE_POS));

    // Make sure the arm will go normal speed again when it exits these states
    addTransition(
        NEW_GROUND_PICKUP, SEEKING_STOWED, setArmNormalSpeedCommand().alongWith(enableClawProx()));
    addTransition(
        NEW_GROUND_INTERMEDIATE,
        SEEKING_STOWED,
        setArmNormalSpeedCommand().alongWith(enableClawProx()));

    addTransition(STOWED, LOW_SCORE, () -> goToArmState(LOW_POS));
    addTransition(PRIMED, LOW_SCORE, () -> goToArmState(LOW_POS));
    addTransition(STOWED, MID_SCORE, () -> goToArmState(MID_POS));
    addTransition(PRIMED, MID_SCORE, () -> goToArmState(MID_POS));
    addTransition(STOWED, HIGH_CUBE, () -> goToArmState(HIGH_CUBE_POS));
    addTransition(PRIMED, HIGH_CUBE, () -> goToArmState(HIGH_CUBE_POS));

    addTransition(STOWED, SEEKING_POSE);
    addTransition(SEEKING_POSE, AT_POSE);

    addTransition(PICKUP_GROUND, SEEKING_HIGH);

    addTransition(STOWED, SEEKING_PICKUP_DOUBLE, claw.transitionCommand(ClawState.OPENED));
    addTransition(SEEKING_PICKUP_DOUBLE, SEEKING_HIGH);

    addTransition(STOWED, SEEKING_HIGH);
    addTransition(PRIMED, SEEKING_HIGH);
    removeTransition(SEEKING_HIGH, STOWED);
    addTransition(SEEKING_HIGH, HIGH);
    addTransition(HIGH_CUBE, SEEKING_PICKUP_GROUND);

    addTransition(
        PICKUP_GROUND, SEEKING_STOWED, new InstantCommand(() -> setWristTarget(toRadians(-45))));
    addTransition(HIGH, SEEKING_STOWED, () -> setWristTarget(toRadians(-45)));

    addTransition(STOWED, SEEKING_PICKUP_GROUND);
    addTransition(HIGH, SEEKING_PICKUP_GROUND);
    removeTransition(SEEKING_PICKUP_GROUND, STOWED);
    addTransition(SEEKING_PICKUP_GROUND, PICKUP_GROUND);
    addTransition(HIGH_CUBE, SEEKING_PICKUP_GROUND);
  }

  private void registerStateCommands() {
    registerStateCommand(
        SEEKING_HIGH, new ExtendArmCommand(this, HIGH_POS).andThen(transitionCommand(HIGH)));

    registerStateCommand(
        SEEKING_PICKUP_GROUND,
        new ExtendArmCommand(this, GROUND_PICKUP_POS).andThen(transitionCommand(PICKUP_GROUND)));

    registerStateCommand(
        SEEKING_PICKUP_DOUBLE,
        new ExtendArmCommand(this, PICKUP_DOUBLE_POS).andThen(transitionCommand(PICKUP_DOUBLE)));

    registerStateCommand(
        SEEKING_STOWED,
        new FunctionalCommand(
                () -> {
                  if (getWristAngle() < 0
                      && getShoulderAngle() < toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT))
                    setWristTarget(0);
                  if (getShoulderAngle() < 0) setShoulderTarget(toRadians(15));
                },
                () -> {},
                (interrupted) -> {},
                () ->
                    getShoulderAngle() >= 0
                        && (getShoulderAngle() >= toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT)
                            || getWristAngle() >= 0))
            .andThen(
                new FunctionalCommand(
                    () -> {
                      setElevatorTarget(STOWED_POS.getElevatorExtension());
                      setShoulderTarget(toRadians(45));
                    },
                    () -> {},
                    (interrupted) -> {},
                    () -> getError(getElevatorHeight(), getElevatorTarget()) <= ELEVATOR_TOLERANCE),
                new InstantCommand(() -> goToArmState(STOWED_POS)),
                new InstantCommand(() -> requestTransition(STOWED))));

    registerStateCommand(
        SEEKING_PRIMED,
        new FunctionalCommand(
                () -> {
                  if (getWristAngle() < 0
                      && getShoulderAngle() < toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT))
                    setWristTarget(0);
                  if (getShoulderAngle() < 0) setShoulderTarget(toRadians(15));
                },
                () -> {},
                (interrupted) -> {},
                () ->
                    getShoulderAngle() >= 0
                        && (getShoulderAngle() >= toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT)
                            || getWristAngle() >= 0))
            .andThen(
                new FunctionalCommand(
                    () -> {
                      setElevatorTarget(PRIMED_POS.getElevatorExtension());
                      setShoulderTarget(toRadians(-45));
                    },
                    () -> {},
                    (interrupted) -> {},
                    () -> getError(getElevatorHeight(), getElevatorTarget()) <= ELEVATOR_TOLERANCE),
                new InstantCommand(() -> goToArmState(PRIMED_POS)),
                new InstantCommand(() -> requestTransition(PRIMED))));

    registerStateCommand(
        SEEKING_POSE,
        new FunctionalCommand(
                () -> {
                  //                setRotatorTarget(currentArmState.getRotatorAngle());
                  setWristTarget(currentArmState.getWristAngle());
                  setShoulderTarget(currentArmState.getShoulderAngle());
                },
                () -> {},
                (interrupted) -> {},
                () -> getShoulderAngle() <= SHOULDER_ELEVATOR_THRESHOLD)
            .andThen(
                new InstantCommand(() -> setElevatorTarget(currentArmState.getElevatorExtension())),
                new InstantCommand(() -> requestTransition(AT_POSE))));
  }

  public enum ArmMode {
    UNDETERMINED,
    SEEKING_STOWED,
    STOWED,
    SEEKING_PICKUP_DOUBLE,
    PICKUP_DOUBLE,
    SEEKING_PICKUP_GROUND,
    PICKUP_GROUND,
    NEW_GROUND_PICKUP,
    NEW_GROUND_INTERMEDIATE,
    TELEOP_GROUND_INTERMEDIATE,
    LOW_SCORE,
    MID_SCORE,
    SEEKING_HIGH,
    HIGH,
    HIGH_CUBE,
    SEEKING_PRIMED,
    PRIMED,
    SOFT_STOP,

    SEEKING_POSE,
    AT_POSE,

    TESTING
  }

  public Command calculateElevatorFF(Trigger increment, BooleanSupplier interrupt) {
    return io.calculateElevatorFF(increment, interrupt);
  }

  public Command calculateWristFF(Trigger increment, BooleanSupplier interrupt) {
    return io.calculateWristFF(increment, interrupt);
  }

  public Command calculateShoulderFF(Trigger increment, BooleanSupplier interrupt) {
    return io.calculateShoulderFF(increment, interrupt);
  }

  public InstantCommand reset() {
    return new InstantCommand(this::pollAbsoluteAngles);
  }

  public Command runTrajectory(Pose3d endPose, double time) {
    return new ArmTrajectory(kinematics, getArmState(), endPose, time).run(this::goToArmState);
  }

  public void goToArmState(ArmState state) {
    if (state.isValid()) {
      setElevatorTarget(state.getElevatorExtension());
      setShoulderTarget(state.getShoulderAngle());
      setWristTarget(state.getWristAngle());
    }
  }

  public void goToPose(Pose3d pose) {
    ArmState target = runIK(pose);

    System.out.println(target);

    if (target != null) {
      System.out.println(target);

      currentArmState = target;
      requestTransition(SEEKING_POSE);
      // goToArmState(target);
    }
  }

  public ArmState runIK(Pose3d pose) {
    ArmState[] solutions = kinematics.inverseKinematics(pose);

    return kinematics.chooseIdealState(getArmState(), solutions);
  }

  public Pose3d forwardKinematics(ArmState state) {
    return kinematics.forwardKinematics(state);
  }

  /**
   * Set the target of the elevator
   *
   * @param target target height (in meters)
   */
  public void setElevatorTarget(double target) {
    io.setElevatorTarget(target);
  }

  /**
   * Set the target of the shoulder
   *
   * @param target target angle (in radians)
   */
  public void setShoulderTarget(double target) {
    io.setShoulderTarget(target);
  }

  /**
   * Set the target of the wrist
   *
   * @param target target angle (in radians)
   */
  public void setWristTarget(double target) {
    io.setWristTarget(target);
  }

  @Override
  protected void determineSelf() {
    pollAbsoluteAngles();

    setArmNormalSpeed();

    // setState(STOWED);
    requestTransition(ArmMode.SEEKING_STOWED);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);

    Logger.processInputs(this.getName(), inputs);
  }

  @Override
  protected void onEnable() {

    setElevatorTarget(getElevatorHeight());

    setArmNormalSpeed();
  }

  @Override
  protected void onDisable() {}

  public Pose3d getArmPose() {
    return kinematics.forwardKinematics(getArmState());
  }

  public ArmState getArmState() {
    return new ArmState(getElevatorHeight(), getShoulderAngle(), getWristAngle());
  }

  public Claw claw() {
    return claw;
  }

  public double getElevatorHeight() {
    return inputs.elevatorPos;
  }

  public double getElevatorTarget() {
    return inputs.elevatorTarget;
  }

  public double getShoulderAngle() {
    return inputs.shoulderAbsolutePos;
  }

  public double getShoulderTarget() {
    return inputs.shoulderTarget;
  }

  public double getWristAngle() {
    return inputs.wristAbsolutePos;
  }

  public double getWristTarget() {
    return inputs.wristTarget;
  }

  private double getError(double num1, double num2) {
    return abs(num1 - num2);
  }

  public boolean isShoulderSafe() {
    return getElevatorHeight() <= ELEVATOR_TOLERANCE;
  }

  public boolean isElevatorSafe() {
    return getShoulderAngle() <= SHOULDER_ELEVATOR_THRESHOLD;
  }

  public void pollAbsoluteAngles() {
    System.out.println(inputs.shoulderAbsolutePos);
    System.out.println(inputs.wristAbsolutePos);

    io.resetShoulderPos(inputs.shoulderAbsolutePos);

    io.resetWristPos(inputs.wristAbsolutePos);
  }

  public void setArmSlowSpeed() {
    io.changeShoulderSpeed(SHOULDER_SLOW_VEL, SHOULDER_SLOW_ACCEL, SHOULDER_SLOW_JERK);
    io.changeWristSpeed(WRIST_SLOW_VEL, WRIST_SLOW_ACCEL, WRIST_SLOW_JERK);
  }

  public Command setArmSlowSpeedCommand() {
    return new InstantCommand(this::setArmSlowSpeed);
  }

  public void setArmNormalSpeed() {
    io.changeShoulderSpeed(SHOULDER_VEL, SHOULDER_ACCEL, SHOULDER_JERK);
    io.changeWristSpeed(WRIST_VEL, WRIST_ACCEL, WRIST_JERK);
  }

  public Command setArmNormalSpeedCommand() {
    return new InstantCommand(this::setArmNormalSpeed);
  }

  public void setArmFastSpeed() {
    io.changeShoulderSpeed(SHOULDER_FAST_VEL, SHOULDER_FAST_ACCEL, SHOULDER_FAST_JERK);
    io.changeWristSpeed(WRIST_VEL, WRIST_ACCEL, WRIST_JERK);
  }

  public Command setArmFastSpeedCommand() {
    return new InstantCommand(this::setArmFastSpeed);
  }

  public Pose3d getElevatorPose(double turretRot) {
    return new Pose3d(0, 0, getElevatorHeight(), new Rotation3d(Math.PI / 2, 0, turretRot));
  }

  public Pose3d getElevatorPoseTarget(double turretRot) {
    return new Pose3d(0, 0, getElevatorTarget(), new Rotation3d(Math.PI / 2, 0, turretRot));
  }

  public Pose3d getShoulderPose(double turretRot) {
    return new Pose3d(
        0,
        0,
        0.53 + getElevatorHeight(),
        new Rotation3d(Math.PI / 2 - getShoulderAngle(), 0, turretRot));
  }

  public Pose3d getShoulderPoseTarget(double turretRot) {
    return new Pose3d(
        0,
        0,
        0.53 + getElevatorTarget(),
        new Rotation3d(Math.PI / 2 - getShoulderTarget(), 0, turretRot));
  }

  public Pose3d getWristPose(double turretRot) {
    double shoulderLength = Units.inchesToMeters(28.765575);
    double shoulderProtusion = shoulderLength * cos(getShoulderAngle());
    return new Pose3d(
        shoulderProtusion * sin(turretRot),
        -shoulderProtusion * cos(turretRot),
        0.53 + getElevatorHeight() + shoulderLength * sin(getShoulderAngle()),
        new Rotation3d(Math.PI / 2 - getShoulderAngle() - getWristAngle(), 0, turretRot));
  }

  public Pose3d getWristPoseTarget(double turretRot) {
    double shoulderLength = Units.inchesToMeters(28.765575);
    double shoulderProtusion = shoulderLength * cos(getShoulderTarget());
    return new Pose3d(
        shoulderProtusion * sin(turretRot),
        -shoulderProtusion * cos(turretRot),
        0.53 + getElevatorTarget() + shoulderLength * sin(getShoulderTarget()),
        new Rotation3d(Math.PI / 2 - getShoulderTarget() - getWristTarget(), 0, turretRot));
  }

  @Override
  protected void additionalSendableData(SendableBuilder builder) {

    // builder.addDoubleProperty("elevator/output", () -> Units.metersToInches(getElevatorHeight()),
    // null);
    builder.addDoubleProperty(
        "elevator/height", () -> Units.metersToInches(getElevatorHeight()), null);
    builder.addDoubleProperty(
        "elevator/target", () -> Units.metersToInches(getElevatorTarget()), null);
    // builder.addDoubleProperty("elevator/error", () ->
    // getError(Units.metersToInches(getElevatorTarget()),
    // Units.metersToInches(getElevatorHeight())), null);

    // builder.addDoubleProperty("shoulder/velo", () -> toDegrees(shoulder.getEncoderVelocity()),
    // null);
    builder.addDoubleProperty(
        "shoulder/motor-angle", () -> toDegrees(inputs.shoulderMotorPos), null);
    builder.addDoubleProperty("shoulder/target", () -> toDegrees(getShoulderTarget()), null);
    builder.addDoubleProperty(
        "shoulder/absolute-angle", () -> toDegrees(inputs.shoulderAbsolutePos), null);

    builder.addDoubleProperty(
        "shoulder/absolute-error",
        () -> getError(inputs.shoulderAbsolutePos, toDegrees(getShoulderTarget())),
        null);
    builder.addDoubleProperty(
        "shoulder/motor-error",
        () -> getError(toDegrees(inputs.shoulderMotorPos), toDegrees(getShoulderTarget())),
        null);

    // builder.addDoubleProperty("shoulder/shoulder-target-velo", () ->
    // toDegrees(shoulderPID.getSetpoint().velocity), null);
    // builder.addDoubleProperty("shoulder/shoulder-target-pos", () ->
    // toDegrees(shoulderPID.getSetpoint().position), null);

    // builder.addDoubleProperty("wrist/velocity_error", () ->
    // toDegrees(wristPID.getVelocityError()), null);
    // builder.addDoubleProperty("wrist/position_error", () ->
    // toDegrees(wristPID.getPositionError()), null);
    builder.addDoubleProperty(
        "wrist/absolute-angle", () -> toDegrees(inputs.wristAbsolutePos), null);
    builder.addDoubleProperty("wrist/motor-angle", () -> toDegrees(inputs.wristMotorPos), null);
    builder.addDoubleProperty("wrist/target", () -> toDegrees(getWristTarget()), null);
    // builder.addDoubleProperty("wrist/velo", () -> toDegrees(wrist.getEncoderVelocity()), null);

    builder.addDoubleProperty(
        "wrist/absolute-error",
        () -> getError(toDegrees(inputs.wristAbsolutePos), toDegrees(getWristTarget())),
        null);
    builder.addDoubleProperty(
        "wrist/motor-error",
        () -> getError(toDegrees(inputs.wristMotorPos), toDegrees(getWristTarget())),
        null);
  }
}
