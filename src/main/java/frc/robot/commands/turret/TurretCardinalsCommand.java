package frc.robot.commands.turret;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.alliance;
import static java.lang.Math.PI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;
import java.util.function.BooleanSupplier;

public class TurretCardinalsCommand extends Command {

  private final Turret t;
  private final BooleanSupplier towardSupplier;
  private final BooleanSupplier awaySupplier;

  private boolean prevToward;
  private boolean prevAway;

  public TurretCardinalsCommand(
      Turret t, BooleanSupplier towardSupplier, BooleanSupplier awaySupplier) {
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

    if ((currentToward && !prevToward) || (currentAway && !prevAway)) {
      Rotation2d globalTarget = new Rotation2d();
      if (currentToward && alliance == Blue) globalTarget = new Rotation2d(PI);
      if (currentAway && alliance == Red) globalTarget = new Rotation2d(PI);

      // Get the current rotation of the bot to the nearest 90 degrees
      Rotation2d botRotation =
          roundToQuadrantal(Constants.SwerveDrivetrain.getOdoPose.get().getRotation());

      // Get what the target of the turret should be
      Rotation2d turretAngle = globalTarget.minus(botRotation).minus(new Rotation2d(PI / 2));

      System.out.println(
          "evaluating: "
              + turretAngle.getDegrees()
              + " (turret); "
              + botRotation.getDegrees()
              + " (bot rotation)");

      t.setTarget(MathUtil.angleModulus(turretAngle.getRadians()));

      // Back when the turret was broken
      // t.setTarget(currentToward ? Math.toRadians(0) : Math.toRadians(180));
    }

    prevToward = currentToward;
    prevAway = currentAway;
  }

  private Rotation2d roundToQuadrantal(Rotation2d rot) {
    return new Rotation2d(Math.round(rot.getRadians() / (PI / 2)) * PI / 2);
  }
}
