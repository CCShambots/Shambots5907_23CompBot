package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class WhileDisabledInstantCommand extends InstantCommand{

    public WhileDisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    
    
}
