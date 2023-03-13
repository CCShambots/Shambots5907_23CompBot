package frc.robot.commands.auto.blue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class BlueScoreBalanceCenter extends SequentialCommandGroup {

    public BlueScoreBalanceCenter(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                rc.arm().transitionCommand(Arm.ArmMode.HIGH_CUBE),
                new WaitCommand(2),
                rc.arm().openClaw(),
                new InstantCommand(() -> rc.arm().setShoulderTarget(Math.toRadians(75))),
                new WaitCommand(1.5),
                new ParallelCommandGroup(
                        rc.runTraj("blue-dock-center", true),
                        new SequentialCommandGroup(
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED),
                                new WaitCommand(3)
                        )
                ),
                rc.dt().waitForState(Drivetrain.DrivetrainState.IDLE),
                rc.arm().waitForState(Arm.ArmMode.STOWED),
                rc.dt().transitionCommand(Drivetrain.DrivetrainState.DOCKING)
        );
    }
}
