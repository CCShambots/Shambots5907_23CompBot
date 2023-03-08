package frc.robot.commands.auto.red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class RedScorePickup extends SequentialCommandGroup {

    public RedScorePickup(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                rc.arm().transitionCommand(Arm.ArmMode.HIGH_CUBE),
                new WaitCommand(2),
                rc.arm().openClaw(),
                new ParallelCommandGroup(
                    rc.runTraj("red-pickup-right", true),
                    new SequentialCommandGroup(
                            new WaitCommand(1),
                            rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED)
                    )
                ),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.arm().waitForState(ArmMode.STOWED),
                rc.arm().transitionCommand(ArmMode.SEEKING_PICKUP_GROUND),
                rc.arm().waitForState(ArmMode.PICKUP_GROUND),
                new WaitCommand(3),
                rc.arm().closeClaw(),
                new WaitCommand(0.5),
                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED)
        );
    }
}
