package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

        if((currentToward && !prevToward) || (currentAway && !prevAway)) {
            Rotation2d globalTarget = new Rotation2d();
            if(currentToward && alliance == Blue) globalTarget = new Rotation2d(PI);
            if(currentAway && alliance == Red) globalTarget = new Rotation2d(PI);

            //Get the current rotation of the bot to the nearest 90 degrees
            Rotation2d botRotation = roundToQuadrantal(Constants.SwerveDrivetrain.getOdoPose.get().getRotation());

            //Get what the target of the turret should be
            Rotation2d turretAngle = globalTarget.minus(botRotation).minus(new Rotation2d(PI/2));

            System.out.println("evaluating: " + turretAngle.getDegrees() + " (turret); " + botRotation.getDegrees() + " (bot rotation)");

            t.setTarget(MathUtil.angleModulus(turretAngle.getRadians()));
        }

        prevToward = currentToward;
        prevAway = currentAway;

    }

    private Rotation2d getCurrentCardinal(Translation2d point) {
        Pose2d botPose = Constants.SwerveDrivetrain.getOdoPose.get();
        return new Rotation2d(Math.round(Math.atan2(point.getY() - botPose.getY(), point.getX() - botPose.getX()) / (PI / 2)) * (PI / 2));
    }

    private Rotation2d roundToQuadrantal(Rotation2d rot) {
        return new Rotation2d(Math.round(rot.getRadians() / (PI / 2)) * PI / 2);
    }

}
