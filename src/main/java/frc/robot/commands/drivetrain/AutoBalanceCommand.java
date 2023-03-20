package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.subsystems.Drivetrain;

import java.util.*;
import java.util.function.IntSupplier;

public class AutoBalanceCommand extends CommandBase {
    private int direction;
    private final IntSupplier directionSupplier;
    private final Drivetrain dt;

    private final PIDController pid;

    private final ArrayList<Double> buff;

    private int rMod, pMod;

    public AutoBalanceCommand(Drivetrain dt, IntSupplier directionSupplier, PIDGains pidGains, int bufferSize) {
        this.directionSupplier = directionSupplier;
        this.dt = dt;

        pid = new PIDController(pidGains.p, pidGains.i, pidGains.d, 0.02);
        buff = new ArrayList<>(Collections.nCopies(bufferSize, getCumulativeAngle()));

        defineMods();
    }

    private void defineMods() {
        rMod = dt.getRoll() < 0 ? 1 : -1;
        pMod = dt.getPitch() < 0 ? 1 : -1;
    }

    private double getCumulativeAngle() {
        return dt.getPitch() * pMod + dt.getRoll() * rMod;
    }

    @Override
    public void initialize() {
        pid.reset();
        this.direction = Math.max(Math.min(1, directionSupplier.getAsInt()), -1);
    }

    @Override
    public void execute() {
        buff.remove(buff.size() - 1);
        buff.add(0, getCumulativeAngle());

        ChassisSpeeds speeds = new ChassisSpeeds(
                Constants.SwerveDrivetrain.AUTO_BALANCE_SPEED * direction * pid.calculate(getCumulativeAngle(), 0),
                0,
                0
        );

        dt.drive(speeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        dt.stopModules();
    }

    @Override   
    public boolean isFinished() {
        Optional<Double> v = buff.stream().max(Comparator.naturalOrder());

        //3 is how many cumulative degrees of incline it should be less than for however long the buffer is
        //can be changed to "v.isPresent() && v.get() < 3" but this is easier to read
        return v.isPresent() ? v.get() < 3 : false;
    }
}
