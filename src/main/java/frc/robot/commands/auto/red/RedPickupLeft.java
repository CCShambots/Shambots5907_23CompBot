package frc.robot.commands.auto.red;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.arm.Arm.ArmMode;

public class RedPickupLeft extends BaseAutoRoute {

  public RedPickupLeft(RobotContainer rc) {
    super(Alliance.Red);

    addCommands(
        rc.waitForReady(),
        new ScoreFirstElementCommand(rc),
        new ParallelCommandGroup(
            rc.runTraj("red-pickup-left", true),
            new SequentialCommandGroup(
                new WaitCommand(1),
                rc.arm().setArmNormalSpeedCommand(),
                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
                //                                rc.arm().waitForState(ArmMode.PRIMED),
                new WaitCommand(0.5),
                rc.turret().goToAngle(Math.toRadians(90)),
                new WaitCommand(0.5)
                //                                rc.arm().setArmSlowSpeedCommand(),
                //                                rc.arm().openClaw(),
                //
                // rc.arm().transitionCommand(Arm.ArmMode.SEEKING_PICKUP_GROUND),
                //                                new WaitCommand(2.5),
                //
                // rc.turret().transitionCommand(TurretState.INTAKING)
                ))
        //                rc.dt().waitForState(DrivetrainState.IDLE),
        //                rc.arm().closeClaw(),
        //                new WaitCommand(0.5),
        //                rc.cv().transitionCommand(VisionState.ELEMENT_TYPE),
        //                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        //                rc.turret().transitionCommand(TurretState.IDLE),
        //                rc.arm().waitForState(ArmMode.STOWED),
        //                new WaitCommand(1),
        //                new ConditionalCommand(
        //                        rc.runTraj("red-return-left").alongWith(
        //                                rc.turret().goToAngle(Math.toRadians(-90)),
        //                                rc.arm().setArmNormalSpeedCommand()
        //                        )
        //                , new InstantCommand(),
        //                        () -> rc.cv().getCurrentElementType() == ElementType.Cone)
        );
  }
}
