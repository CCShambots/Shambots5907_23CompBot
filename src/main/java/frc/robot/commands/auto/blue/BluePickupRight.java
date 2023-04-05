package frc.robot.commands.auto.blue;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElementType;
import frc.robot.commands.auto.BonkShot;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Turret.TurretState;

public class BluePickupRight extends SequentialCommandGroup {

    public BluePickupRight(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                new ScoreFirstElementCommand(rc),

                rc.cv().transitionCommand(VisionState.CONE_DETECTOR),
                new ParallelCommandGroup(
                        rc.runTraj("blue-pickup-right", true),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                rc.arm().setArmNormalSpeedCommand(),
                                rc.arm().transitionCommand(ArmMode.SEEKING_PRIMED),
                                rc.arm().waitForState(ArmMode.PRIMED),
                                new WaitCommand(0.5),
                                rc.turret().goToAngle(Math.toRadians(-90)),
                                new WaitCommand(0.5),
                                rc.arm().setArmSlowSpeedCommand(),
                                rc.arm().openClaw(),
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_PICKUP_GROUND),
                                new WaitCommand(2.5),
                                rc.turret().transitionCommand(TurretState.INTAKING)
                        )
                ),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.arm().closeClaw(),
                new WaitCommand(0.5),
                rc.cv().transitionCommand(VisionState.ELEMENT_TYPE),
                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
                rc.turret().transitionCommand(TurretState.IDLE),
                rc.arm().waitForState(ArmMode.STOWED),
                new WaitCommand(1),
                new ConditionalCommand(
                        rc.runTraj("blue-return-right").alongWith(
                                rc.turret().goToAngle(Math.toRadians(90)),
                                rc.arm().setArmNormalSpeedCommand()
                        )
                , new InstantCommand(), 
                        () -> rc.cv().getCurrentElementType() == ElementType.Cone)
        );
    }
}
