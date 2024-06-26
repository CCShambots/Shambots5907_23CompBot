package frc.robot.subsystems.turret;

import static frc.robot.Constants.Turret.*;
import static frc.robot.Constants.Vision.BASE_HAS_TARGET_SUPPLIER;
import static frc.robot.Constants.Vision.BASE_X_OFFSET_SUPPLIER;
import static frc.robot.subsystems.turret.Turret.TurretState.*;
import static java.lang.Math.*;
import static java.lang.Math.toDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.WhileDisabledInstantCommand;
import frc.robot.commands.turret.NewTurretManualControlCommand;
import frc.robot.commands.turret.TurretCardinalsCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends StateMachine<Turret.TurretState> {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final BooleanSupplier towardSupplier;
  private final BooleanSupplier awaySupplier;
  private final BooleanSupplier leftSupplier;
  private final BooleanSupplier rightSupplier;

  private final BooleanSupplier clawHasTarget;
  private final DoubleSupplier clawVisionOffset;

  private boolean enforceStartAngle = false;
  private double startAngle = 0;

  /**
   * @param io
   * @param towardSupplier
   * @param awaySupplier
   * @param clawHasTarget Remnant from claw vision code
   * @param clawVisionOffset Remnant from old claw vision code
   * @param leftSupplier
   * @param rightSupplier
   */
  public Turret(
      TurretIO io,
      BooleanSupplier towardSupplier,
      BooleanSupplier awaySupplier,
      BooleanSupplier clawHasTarget,
      DoubleSupplier clawVisionOffset,
      BooleanSupplier leftSupplier,
      BooleanSupplier rightSupplier) {

    super("turret", UNDETERMINED, TurretState.class);

    this.io = io;

    this.towardSupplier = towardSupplier;
    this.awaySupplier = awaySupplier;

    this.clawHasTarget = clawHasTarget;
    this.clawVisionOffset = clawVisionOffset;

    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;

    defineTransitions();
    registerStateCommands();

    new WaitCommand(1).andThen(new WhileDisabledInstantCommand(this::pullAbsoluteAngle)).schedule();
    // pullAbsoluteAngle();
  }

  private void defineTransitions() {
    addOmniTransition(
        IDLE, new InstantCommand(() -> io.changeSpeed(TURRET_MAX_VEL, TURRET_MAX_ACCEL, 2500)));
    addOmniTransition(
        SCORING, new InstantCommand(() -> io.changeSpeed(TURRET_MAX_VEL, TURRET_MAX_ACCEL, 2500)));
    addOmniTransition(
        INTAKING,
        new InstantCommand(() -> io.changeSpeed(TURRET_SLOW_VEL, TURRET_SLOW_ACCEL, 1000)));
    addOmniTransition(
        CARDINALS,
        new InstantCommand(() -> io.changeSpeed(TURRET_MAX_VEL, TURRET_MAX_ACCEL, 2500)));
    addOmniTransition(
        MANUAL_CONTROL,
        new InstantCommand(() -> io.changeSpeed(TURRET_MAX_VEL, TURRET_MAX_ACCEL, 2500)));

    addOmniTransition(
        LIMELIGHT_SCORING,
        new InstantCommand(() -> io.changeSpeed(TURRET_MAX_VEL, TURRET_MAX_ACCEL, 2500)));

    addOmniTransition(SOFT_STOP, new InstantCommand(() -> io.setPower(0)));
  }

  private void registerStateCommands() {
    registerStateCommand(
        CARDINALS,
        new InstantCommand(() -> setTarget(getTurretAngle()))
            .andThen(new TurretCardinalsCommand(this, towardSupplier, awaySupplier)));

    // registerStateCommand(SCORING, new RunCommand(() -> {
    //
    // setTargetToPoint(Constants.gridInterface.getNextElement().getLocation().toTranslation2d());
    // }));

    registerStateCommand(
        INTAKING,
        new RunCommand(
            () -> {
              if (!isBusy() && clawHasTarget.getAsBoolean()) {
                double clawOffset = clawVisionOffset.getAsDouble();

                double mult = AIMING_LUT.getClosest(clawOffset);

                if (Math.abs(clawOffset) < 3) {

                  setTarget(getRelativeAngle() + mult * clawVisionOffset.getAsDouble());
                }
              }
            }));

    registerStateCommand(
        LIMELIGHT_SCORING,
        new RunCommand(
            () -> {
              if (!isBusy() && BASE_HAS_TARGET_SUPPLIER.getAsBoolean()) {
                double baseOffset = BASE_X_OFFSET_SUPPLIER.getAsDouble();

                double mult = AIMING_LUT.getClosest(baseOffset);

                if (Math.abs(baseOffset) < 3) {
                  setTarget(getRelativeAngle() + mult * BASE_X_OFFSET_SUPPLIER.getAsDouble());
                }
              }
            }));

    registerStateCommand(
        MANUAL_CONTROL, new NewTurretManualControlCommand(this, leftSupplier, rightSupplier));
  }

  public enum TurretState {
    UNDETERMINED,
    IDLE,
    CARDINALS,
    SCORING,
    LIMELIGHT_SCORING,
    INTAKING,
    SOFT_STOP,
    MANUAL_CONTROL
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);
  }

  public Pose3d getOffsetPose() {
    return new Pose3d(0, 0, 0, new Rotation3d(Math.PI / 2, 0, getTurretAngle()));
  }

  public Pose3d getOffsetTargetPose() {
    return new Pose3d(0, 0, 0, new Rotation3d(Math.PI / 2, 0, getTurretTarget()));
  }

  @Override
  protected void determineSelf() {
    pullAbsoluteAngle();
    setTarget(getRelativeAngle());
    //        setTarget(TURRET_START_ANGLE); //TODO: no worries right
    setState(IDLE);
  }

  public void setTargetToPoint(Translation2d target) {
    setTarget(getTurretAngleToPoint(target).getRadians());
  }

  public Command calculateFF(Trigger increment, BooleanSupplier interrupt) {
    return io.calculateFF(increment, interrupt);
  }

  /**
   * Get the angle the turret would need to face to be facing towards a point in field space
   *
   * @param target the target position in field space
   * @return the angle the turret would be at
   */
  public Rotation2d getTurretAngleToPoint(Translation2d target) {
    Pose2d current = Constants.SwerveDrivetrain.getOdoPose.get();

    Translation2d relative = target.minus(current.getTranslation());

    Rotation2d angle =
        new Rotation2d(Math.atan2(relative.getY(), relative.getX()))
            .minus(current.getRotation())
            .minus(Rotation2d.fromDegrees(90));
    return angle;
  }

  public double getTurretAngle() {
    return inputs.potentiometerAngle;
  }

  public double getRelativeAngle() {
    return inputs.turretMotorAngle;
  }

  public double getTurretTarget() {
    return inputs.turretTarget;
  }

  public double getErorr() {
    return Math.abs(getTurretAngle() - getTurretTarget());
  }

  public void pullAbsoluteAngle() {
    io.resetMotorAngle(getTurretAngle());
  }

  public double getMinimumAbsoluteErrorToStartingPos() {
    double deg = toDegrees(getTurretAngle());
    return min(abs(deg - 90), abs(deg + 90));
  }

  public void resetAngle(double angle /*radians*/) {
    io.resetMotorAngle(angle);
  }

  public boolean isBusy() {
    return getErorr() > TURRET_ALLOWED_ERROR;
  }

  public Command setStartAngle(double angle) {
    return new InstantCommand(
        () -> {
          enforceStartAngle = true;
          startAngle = angle;
        });
  }

  /**
   * Set the target of the turret
   *
   * @param target target angle (in radians)
   */
  public void setTarget(double target) {
    if (TURRET_RANGE.isWithin(target)) {
      io.setTarget(target);
    }
  }

  public Command goToAngle(double target) {
    return new InstantCommand(() -> setTarget(target));
  }

  @Override
  protected void onTeleopStart() {
    if (enforceStartAngle) {
      setTarget(startAngle);
      enforceStartAngle = false;
    }
  }
}
