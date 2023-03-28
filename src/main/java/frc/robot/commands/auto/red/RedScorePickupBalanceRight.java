package frc.robot.commands.auto.red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.gridInterface;
import static java.lang.Math.toRadians;

public class RedScorePickupBalanceRight extends SequentialCommandGroup {

    public RedScorePickupBalanceRight(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                rc.arm().setArmSlowSpeedCommand(),
                rc.arm().transitionCommand(ArmMode.HIGH_CUBE),
                new WaitCommand(2),
                rc.arm().openClaw(),
                new ParallelCommandGroup(
                        rc.runTraj("red-get-element-right", true),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                gridInterface.setNextElementCommand(2, 6),
                                rc.turret().goToAngle(toRadians(90)),
                                rc.arm().setArmNormalSpeedCommand(),
                                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
                                new WaitCommand(2),
                                rc.arm().setArmSlowSpeedCommand(),
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_PICKUP_GROUND),
                                new WaitCommand(1.5),
                                rc.turret().transitionCommand(Turret.TurretState.INTAKING)
                        )
                ),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.arm().closeClaw(),
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                    rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
                    rc.runTraj("red-go-balance-right")
                ),
                rc.arm().waitForState(ArmMode.STOWED),
                rc.dt().transitionCommand(DrivetrainState.DOCKING)
        );
    }
}
