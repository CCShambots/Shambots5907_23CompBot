package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.pro.MotionMagicTalonFXPro;
import frc.robot.commands.turret.TurretCardinalsCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.ctre.phoenixpro.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenixpro.signals.NeutralModeValue.Coast;
import static frc.robot.Constants.Turret.*;
import static frc.robot.subsystems.Turret.TurretState.*;
import static java.lang.Math.*;
import static java.lang.Math.toDegrees;

public class Turret extends StateMachine<Turret.TurretState> {

    private final MotionMagicTalonFXPro turret = new MotionMagicTalonFXPro(TURRET_ID, TURRET_GAINS, TURRET_INPUT_TO_OUTPUT, TURRET_MAX_VEL, TURRET_MAX_ACCEL, 2500);
    private final AnalogPotentiometer turretPotentiometer = new AnalogPotentiometer(TURRET_POT_PORT, TURRET_POT_RATIO, TURRET_ENCODER_OFFSET);

    private final BooleanSupplier towardSupplier;
    private final BooleanSupplier awaySupplier;
    private final BooleanSupplier clawHasTarget;
    private final DoubleSupplier clawVisionOffset;

    public Turret(BooleanSupplier towardSupplier, BooleanSupplier awaySupplier, BooleanSupplier clawHasTarget, DoubleSupplier clawVisionOffset) {
        super("turret", UNDETERMINED, TurretState.class);

        this.towardSupplier = towardSupplier;
        this.awaySupplier = awaySupplier;

        this.clawHasTarget = clawHasTarget;
        this.clawVisionOffset = clawVisionOffset;

        defineTransitions();
        registerStateCommands();

        turret.configure(Coast, Clockwise_Positive);
        pullAbsoluteAngle();
    }

    private void defineTransitions() {
        addOmniTransition(IDLE);
        addOmniTransition(SCORING);
        addOmniTransition(INTAKING);

        addOmniTransition(SOFT_STOP, new InstantCommand(() -> turret.set(0)));
    }

    private void registerStateCommands() {
        registerStateCommand(CARDINALS, new TurretCardinalsCommand(this, towardSupplier, awaySupplier));

        registerStateCommand(SCORING, new RunCommand(() -> {
            setTargetToPoint(Constants.gridInterface.getNextElement().getLocation().toTranslation2d());
        }));

        registerStateCommand(INTAKING, new RunCommand(() -> {
                if(clawHasTarget.getAsBoolean()) {
                    setTarget(getTurretTarget() + clawVisionOffset.getAsDouble());
                }
        }));
    }

    public enum TurretState {
        UNDETERMINED,
        IDLE,
        CARDINALS,
        SCORING,
        INTAKING,
        SOFT_STOP
    }

    @Override
    protected void determineSelf() {
        pullAbsoluteAngle();
        setTarget(TURRET_START_ANGLE);
        setState(IDLE);
    }

    public void setTargetToPoint(Translation2d target) {
        setTarget(getTurretAngleToPoint(target).getRadians());
    }

    /**
     * Get the angle the turret would need to face to be facing towards a point in field space
     * @param target the target position in field space
     * @return the angle the turret would be at
     */
    public Rotation2d getTurretAngleToPoint(Translation2d target) {
        Pose2d current = Constants.SwerveDrivetrain.getOdoPose.get();

        Translation2d relative = target.minus(current.getTranslation());

        Rotation2d angle = new Rotation2d(Math.atan2(relative.getY(), relative.getX())).minus(current.getRotation()).minus(Rotation2d.fromDegrees(90));
        return angle;
    }

    public double getTurretAngle() {
        return toRadians(turretPotentiometer.get());
    }

    public double getTurretTarget() {
        return turret.getTarget();
    }

    public void pullAbsoluteAngle() {
        turret.resetPosition(turretPotentiometer.get() * (PI / 180));
    }

    /**
     * Set the target of the turret
     * @param target target angle (in radians)
     */
    public void setTarget(double target) {
        if(TURRET_RANGE.isWithin(target)) {
            turret.setTarget(target);
        }
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleProperty("turret/angle", () -> toDegrees(getTurretAngle()), null);
        builder.addDoubleProperty("turret/target", () -> toDegrees(getTurretTarget()), null);
        builder.addDoubleProperty("turret/error", () -> Math.abs(toDegrees(getTurretTarget()) -  toDegrees((getTurretAngle()))), null);
        // builder.addDoubleProperty("turret/absolute", () -> turretPotentiometer.get(), null);
    }
}
