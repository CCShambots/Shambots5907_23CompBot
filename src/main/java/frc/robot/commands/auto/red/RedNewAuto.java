package frc.robot.commands.auto.red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

import static frc.robot.Constants.gridInterface;
import static java.lang.Math.toRadians;

public class RedNewAuto extends SequentialCommandGroup {

    public RedNewAuto(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                gridInterface.setNextElementCommand(2, 7),
                rc.arm().transitionCommand(Arm.ArmMode.HIGH_CUBE),
                new WaitCommand(2),
                rc.arm().openClaw(),
                gridInterface.indicateElementPlacedCommand(2, 7),
                new ParallelCommandGroup(
                        rc.runTraj("red-get-element-right", true),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                gridInterface.setNextElementCommand(2, 6),
                                rc.turret().goToAngle(toRadians(90)),
                                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
                                new WaitCommand(2),
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_PICKUP_GROUND),
                                new WaitCommand(1.5),
                                rc.turret().transitionCommand(Turret.TurretState.INTAKING)
                        )
                ),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.arm().closeClaw(),
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                        rc.runTraj("red-go-score-right"),
                        new SequentialCommandGroup(
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED),
                                rc.turret().transitionCommand(Turret.TurretState.IDLE),
                                new WaitCommand(2),
                                rc.turret().goToAngle(toRadians(-90)),
                                new WaitCommand(2),
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_HIGH)
                        )
                ),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.arm().waitForState(Arm.ArmMode.HIGH),
                rc.arm().openClaw(),
                gridInterface.indicateElementPlacedCommand(2, 6),
                new ParallelCommandGroup(
                    rc.runTraj("red-balance-right"),
                    new SequentialCommandGroup(
                            new WaitCommand(1.5),
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
