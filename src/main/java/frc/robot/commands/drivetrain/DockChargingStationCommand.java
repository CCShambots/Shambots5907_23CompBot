package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.IntSupplier;

public class DockChargingStationCommand extends Command {
  private int direction;
  private final IntSupplier directionSupplier;
  private final Drivetrain dt;

  private double dockThreshold = Constants.SwerveDrivetrain.AutoBalance.DOCK_THRESHOLD;

  public DockChargingStationCommand(Drivetrain dt, IntSupplier directionSupplier) {

    this.directionSupplier = directionSupplier;
    this.dt = dt;
  }

  public DockChargingStationCommand(
      Drivetrain dt, IntSupplier directionSupplier, double dockThreshold) {
    this(dt, directionSupplier);

    this.dockThreshold = dockThreshold;
  }

  @Override
  public void initialize() {
    this.direction = Math.max(Math.min(1, directionSupplier.getAsInt()), -1);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(Constants.SwerveDrivetrain.AutoBalance.DOCK_SPEED * direction, 0, 0),
            dt.getCurrentAngle());
    dt.drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    dt.stopModules();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(dt.getPitch().getDegrees()) + Math.abs(dt.getRoll().getDegrees())
        > dockThreshold;
  }
}
