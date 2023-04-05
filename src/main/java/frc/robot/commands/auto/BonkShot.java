package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.ArmMode;

public class BonkShot extends SequentialCommandGroup{

    public BonkShot(RobotContainer rc) {
        addCommands(
            rc.arm().setArmFastSpeedCommand(),
            rc.arm().transitionCommand(ArmMode.LOW_SCORE),
            new WaitCommand(0.5),
            rc.arm().setArmNormalSpeedCommand()
        );
    }
    
}
