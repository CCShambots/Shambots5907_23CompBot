package frc.robot.commands.arm;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawVision;

import static java.lang.Math.PI;

public class TrackConeAngleCommand extends CommandBase {

    private Arm a;
    private ClawVision cv;
    private LinearFilter filter = LinearFilter.singlePoleIIR(.3, 0.02);

    private double endTolerance = Constants.Arm.END_TOLERANCE_CONE_ANGLE;
    private double angleOffset = Double.MAX_VALUE;

    /**
     * Create a new command to track the cone with the claw
     * The command will finish once the command is locked in
     * @param arm the arm subsystem
     * @param clawVision the clawVision subsystem
     */
    public TrackConeAngleCommand(Arm arm, ClawVision clawVision) {
        this.a = arm;
        this.cv = clawVision;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        angleOffset = Math.IEEEremainder(cv.getConeAngle().getRadians() + PI/2, PI);

        double target = Math.IEEEremainder(angleOffset + a.getRotatorAngle(), 2* PI);

        a.setRotatorTarget(filter.calculate(target));
    }

    @Override
    public void end(boolean interrupted) {
        a.setRotatorTarget(a.getRotatorAngle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(a.getRotatorAngle() - angleOffset) <= endTolerance;
    }
}
