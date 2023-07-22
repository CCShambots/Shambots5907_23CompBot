package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

import java.util.function.BooleanSupplier;

public class NewTurretManualControlCommand extends CommandBase {
    private final BooleanSupplier left;
    private final BooleanSupplier right;

    private boolean prevLeft = false;
    private boolean prevRight= false;

    private final Turret turret;

    public NewTurretManualControlCommand(Turret turret, BooleanSupplier left, BooleanSupplier right) {
        this.left = left;
        this.right = right;

        this.turret = turret;
    }

    

    @Override
    public void initialize() {
        //Set these to true to require me to release the button and repress before the turret bumps at all
        prevLeft = false;
        prevRight = false;

        turret.setTarget(turret.getTurretAngle());
    }



    @Override
    public void execute() {
        boolean curLeft = right.getAsBoolean();
        boolean curRight = left.getAsBoolean();
        //get left and right rotation request in radians per second

        if(curLeft && !prevLeft)  {
            turret.setTarget(turret.getTurretTarget() + Constants.Turret.MANUAL_CONTROL_BUMP);
        }

        if(curRight && !prevRight) {
            turret.setTarget(turret.getTurretTarget() - Constants.Turret.MANUAL_CONTROL_BUMP);
        }

        prevLeft = curLeft;
        prevRight = curRight;
    }
}
