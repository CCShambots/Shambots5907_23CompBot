package frc.robot.commands.auto.red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class RedScorePickupBalanceAuto extends SequentialCommandGroup {

    public RedScorePickupBalanceAuto(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_HIGH),
                rc.arm().waitForState(Arm.ArmMode.HIGH),
                new WaitCommand(1),
                rc.arm().openClaw(),
                new ParallelCommandGroup(
                    rc.runTraj("red-pickup-right", true),
                    new SequentialCommandGroup(
                            new WaitCommand(1),
                            rc.arm().transitionCommand(Arm.ArmMode.SEEKING_PICKUP_GROUND)
                    )
                )
        );
    }
}
