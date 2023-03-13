package frc.robot.commands.auto.blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class BlueScoreBalanceLeft extends SequentialCommandGroup {

    public BlueScoreBalanceLeft(RobotContainer rc) {
        addCommands(
            rc.waitForReady(),
            rc.arm().transitionCommand(ArmMode.HIGH_CUBE),
            new WaitCommand(2),
            rc.arm().openClaw(),
            new ParallelCommandGroup(
                rc.runTraj("blue-dock-left", true),
                new SequentialCommandGroup(
                        new WaitCommand(1),
                        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED)
                )
            ),
            rc.dt().waitForState(DrivetrainState.IDLE),
            rc.arm().waitForState(ArmMode.STOWED),
            rc.dt().transitionCommand(DrivetrainState.DOCKING)
        );
    }
}
