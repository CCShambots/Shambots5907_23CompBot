package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.IntSupplier;

public class DockChargingStationCommand extends CommandBase {
    private int direction;
    private IntSupplier directionSupplier;
    private Drivetrain dt;

    public DockChargingStationCommand(Drivetrain dt, IntSupplier directionSupplier) {

        this.directionSupplier = directionSupplier;
        this.dt = dt;
    }

    @Override
    public void initialize() {
        this.direction = Math.max(Math.min(1, directionSupplier.getAsInt()), -1);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(Constants.SwerveDrivetrain.DOCK_SPEED * direction, 0, 0), 
            dt.getCurrentAngle());
        dt.drive(speeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopModules();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(dt.getPitch()) + Math.abs(dt.getRoll()) > 20;
    }
}
