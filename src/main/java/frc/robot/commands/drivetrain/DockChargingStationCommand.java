package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DockChargingStationCommand extends CommandBase {
    private int direction;
    private Drivetrain dt;

    public DockChargingStationCommand(Drivetrain dt, int direction) {
        addRequirements(dt);

        this.direction = Math.max(Math.min(1, direction), -1);
        this.dt = dt;
    }

    @Override
    public void execute() {
        //TODO: x is long ways right??
        ChassisSpeeds speeds = new ChassisSpeeds(Constants.SwerveDrivetrain.ENGAGE_SPEED * direction, 0, 0);
        dt.drive(speeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopModules();
    }

    @Override
    public boolean isFinished() {
        //TODO: assumes this is in degreese and +-180
        return Math.abs(dt.getPitch() + dt.getRoll()) > 10;
    }
}
