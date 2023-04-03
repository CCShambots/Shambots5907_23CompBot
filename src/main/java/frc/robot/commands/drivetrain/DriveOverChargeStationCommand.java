package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.IntSupplier;

import static frc.robot.Constants.SwerveDrivetrain.AutoBalance.*;

public class DriveOverChargeStationCommand extends CommandBase {
    private final Drivetrain dt;
    private ArrayList<Double> buff;

    private final IntSupplier directionSupplier;

    private int direction;

    public DriveOverChargeStationCommand(Drivetrain dt, IntSupplier directionSupplier) {
        this.directionSupplier = directionSupplier;
        this.dt = dt;
    }

    @Override
    public void initialize() {
        buff = new ArrayList<>(Collections.nCopies(DRIVE_OVER_BUFFER_SIZE, getCumulativeAngle()));
        direction = (int) Math.signum(directionSupplier.getAsInt());
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                Constants.SwerveDrivetrain.AutoBalance.AUTO_BALANCE_SPEED * direction,
                0,
                0
        ), dt.getCurrentAngle());

        dt.drive(speeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopModules();
    }

    private double getCumulativeAngle() {
        return Math.abs(dt.getPitch()) + Math.abs(dt.getRoll());
    }

    @Override
    public boolean isFinished() {
        Optional<Double> v = buff.stream().max(Comparator.naturalOrder());

        return v.isPresent() && v.get() < 2;
    }
}
