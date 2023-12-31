package frc.robot.commands.auto.blue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.arm.Arm.ArmMode;

public class BlueTwoRight extends BaseAutoRoute {

  public BlueTwoRight(RobotContainer rc) {
    super(Alliance.Blue, Math.toRadians(90));

    addCommands(
        rc.waitForReady(),
        new ScoreFirstElementCommand(rc),
        rc.runTraj("blue-get-element-right", true),
        new WaitCommand(1),
        rc.arm().setArmNormalSpeedCommand(),
        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        rc.arm().waitForState(ArmMode.STOWED),
        new WaitCommand(0.5),
        rc.turret().goToAngle(Math.toRadians(-90))

        //                rc.cv().transitionCommand(VisionState.CONE_DETECTOR),
        //                new ParallelCommandGroup(
        //                        rc.runTraj("blue-get-element-right", true),
        //                        new SequentialCommandGroup(
        //                                new WaitCommand(1),
        //                                rc.arm().setArmNormalSpeedCommand(),
        //                                rc.arm().transitionCommand(ArmMode.SEEKING_PRIMED),
        //                                rc.arm().waitForState(ArmMode.PRIMED),
        //                                new WaitCommand(0.5),
        //                                rc.turret().goToAngle(Math.toRadians(-90)),
        //                                new WaitCommand(0.5),
        //                                rc.arm().setArmSlowSpeedCommand(),
        //                                rc.arm().openClaw(),
        //
        // rc.arm().transitionCommand(Arm.ArmMode.NEW_GROUND_INTERMEDIATE),
        //                                new WaitCommand(1.5),
        //                                rc.arm().transitionCommand(ArmMode.NEW_GROUND_PICKUP)
        //                        )
        //                ),
        //                rc.dt().waitForState(DrivetrainState.IDLE),
        //                new GrabSequenceCommand(rc),
        //                rc.arm().transitionCommand(ArmMode.LOW_SCORE),
        //                rc.turret().goToAngle(Math.toRadians(90)),
        //                rc.runTraj("blue-score-element-right"),
        //                new WaitCommand(2.5),
        //                rc.arm().openClaw()
        );
  }
}
