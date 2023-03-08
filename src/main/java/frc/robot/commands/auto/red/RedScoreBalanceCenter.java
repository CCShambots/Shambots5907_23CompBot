package frc.robot.commands.auto.red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class RedScoreBalanceCenter extends SequentialCommandGroup {

    public RedScoreBalanceCenter(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                rc.arm().transitionCommand(Arm.ArmMode.HIGH_CUBE),
                new WaitCommand(2),
                rc.arm().openClaw(),
                new ParallelCommandGroup(
                        rc.runTraj("red-dock-center", true),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED)
                        )
                ),
                rc.dt().waitForState(Drivetrain.DrivetrainState.IDLE),
                rc.arm().waitForState(Arm.ArmMode.STOWED),
                rc.dt().setPositiveDockDirectionCommand(false),
                rc.dt().transitionCommand(Drivetrain.DrivetrainState.DOCKING)
        );
    }
}
