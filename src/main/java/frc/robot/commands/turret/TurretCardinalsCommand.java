package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.alliance;
import static java.lang.Math.PI;

public class TurretCardinalsCommand extends CommandBase {

    private Turret t;
    private BooleanSupplier towardSupplier;
    private BooleanSupplier awaySupplier;

    private boolean prevToward;
    private boolean prevAway;

    public TurretCardinalsCommand(Turret t, BooleanSupplier towardSupplier, BooleanSupplier awaySupplier) {
        this.t = t;
        this.towardSupplier = towardSupplier;
        this.awaySupplier = awaySupplier;
    }

    @Override
    public void initialize() {
        prevToward = false;
        prevAway = false;
    }

    @Override
    public void execute() {
        boolean currentToward = towardSupplier.getAsBoolean();
        boolean currentAway = awaySupplier.getAsBoolean();

        if(currentToward && !prevToward) {
            Rotation2d base = getCurrentCardinal();

            //Flip on blue alliance
            if(alliance == Blue) {
                base = base.plus(new Rotation2d(PI));
            }

            base = base.minus(new Rotation2d(PI/2));

            t.setTarget(base.getRadians());
        }

        else if(currentAway && !prevAway) {
            Rotation2d base = getCurrentCardinal();

            //Flip on red alliance
            if(alliance == Red) {
                base = base.plus(new Rotation2d(PI));
            }

            base = base.minus(new Rotation2d(PI/2));

            t.setTarget(base.getRadians());
        }

        prevToward = currentToward;
        prevAway = currentAway;

    }

    private Rotation2d getCurrentCardinal() {
        return new Rotation2d(Math.round(Constants.SwerveDrivetrain.getOdoPose.get().getRotation().getRadians() / (PI /2)) * PI / 2);
    }

}
