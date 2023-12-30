package frc.robot.commands.drivetrain;

import static frc.robot.Constants.SwerveDrivetrain.AutoBalance.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.IntSupplier;

public class DriveOverChargeStationCommand extends Command {
  private final Drivetrain dt;
  private ArrayList<Double> buff;

  private final IntSupplier directionSupplier;

  // private boolean initialPos = true;

  private int direction;

  public DriveOverChargeStationCommand(Drivetrain dt, IntSupplier directionSupplier) {
    this.directionSupplier = directionSupplier;
    this.dt = dt;
  }

  @Override
  public void initialize() {
    buff = new ArrayList<>(Collections.nCopies(DRIVE_OVER_BUFFER_SIZE, getCumulativeAngle()));
    direction = (int) Math.signum(directionSupplier.getAsInt());

    // initialPos = dt.getPitch() + dt.getRoll() > 0;
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                Constants.SwerveDrivetrain.AutoBalance.DRIVE_OVER_SPEED * direction, 0, 0),
            dt.getCurrentAngle());

    dt.drive(speeds);

    buff.remove(buff.size() - 1);
    buff.add(0, Math.abs(getCumulativeAngle()));
  }

  @Override
  public void end(boolean interrupted) {
    dt.stopModules();
  }

  private double getCumulativeAngle() {
    return Math.abs(dt.getPitch().getDegrees()) + Math.abs(dt.getRoll().getDegrees());
  }

  @Override
  public boolean isFinished() {
    Optional<Double> v = buff.stream().max(Comparator.naturalOrder());

    return v.isPresent() && v.get() < 3;
  }
}
