package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    private Translation2d blueAllianceSide = new Translation2d(0, Units.feetToMeters(14.5));
    private Translation2d redAllianceSide = new Translation2d(Units.feetToMeters(54), Units.feetToMeters(14.5));

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

        if((currentToward && !prevToward) || (currentAway && !prevAway)) {
            Translation2d workingPoint = redAllianceSide;
            if(currentToward && alliance == Blue) workingPoint = blueAllianceSide;
            if(currentAway && alliance == Red) workingPoint = blueAllianceSide;

            Rotation2d base = getCurrentCardinal(workingPoint);

            System.out.println("evaluating: " + base.getDegrees());

            base = base.minus(new Rotation2d(PI/2));

            t.setTarget(base.getRadians());
        }

        prevToward = currentToward;
        prevAway = currentAway;

    }

    private Rotation2d getCurrentCardinal(Translation2d point) {
        return new Rotation2d(Math.round(Math.atan2(point.getY(), point.getX()) / (PI / 2)) * (PI / 2));
    }

}
