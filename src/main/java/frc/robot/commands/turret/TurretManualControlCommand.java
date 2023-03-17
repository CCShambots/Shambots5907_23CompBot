package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

import java.util.function.BooleanSupplier;

public class TurretManualControlCommand extends CommandBase {
    private final BooleanSupplier left;
    private final BooleanSupplier right;

    private final Turret turret;

    public TurretManualControlCommand(Turret turret, BooleanSupplier left, BooleanSupplier right) {
        this.left = left;
        this.right = right;

        this.turret = turret;
    }

    @Override
    public void execute() {
        //get left and right rotation request in radians per second
        double l = ((left.getAsBoolean() ? 1 : 0) * Constants.Turret.MANUAL_CONTROL_VELOCITY) * Constants.Turret.TURRET_INPUT_TO_OUTPUT;
        double r = ((right.getAsBoolean() ? -1 : 0) * Constants.Turret.MANUAL_CONTROL_VELOCITY) * Constants.Turret.TURRET_INPUT_TO_OUTPUT;

        turret.setTarget(turret.getTurretTarget() + (l / 50) + (r / 50)); //divide by 50 to account for 20ms loop
    }
}
