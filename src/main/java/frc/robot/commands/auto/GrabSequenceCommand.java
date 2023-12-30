package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.claw.Claw.ClawState;

public class GrabSequenceCommand extends SequentialCommandGroup{

    public GrabSequenceCommand(RobotContainer rc) {
        addCommands(
            new ConditionalCommand(new SequentialCommandGroup(
                        rc.arm().closeClaw(),
                        new WaitCommand(0.5)
                ), new InstantCommand(), () -> rc.arm().claw().getState() == ClawState.OPENED)
        );
    }
    
}
