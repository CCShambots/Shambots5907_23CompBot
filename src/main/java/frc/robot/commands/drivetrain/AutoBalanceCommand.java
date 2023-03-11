package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.IntSupplier;

public class AutoBalanceCommand extends CommandBase {
    private int direction;
    private IntSupplier directionSupplier;
    private Drivetrain dt;

    private double startTime = 0;

    public AutoBalanceCommand(Drivetrain dt, IntSupplier directionSupplier) {
        addRequirements(dt);

        this.directionSupplier = directionSupplier;

        this.dt = dt;
    }

    @Override
    public void initialize() {
        this.direction = Math.max(Math.min(1, directionSupplier.getAsInt()), -1);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(Constants.SwerveDrivetrain.AUTO_BALANCE_SPEED * direction, 0, 0);
        dt.drive(speeds, false);
    }

    private double getRuntime() {
        return Timer.getFPGATimestamp() - startTime;
    }

    private boolean hasMetMinTime() {
        return getRuntime() > Constants.SwerveDrivetrain.MIN_BALANCE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        startTime = 0;
        dt.stopModules();
    }

    @Override   
    public boolean isFinished() {
        return Math.abs(dt.getPitch() + dt.getRoll()) < 9 && hasMetMinTime();
    }
}
