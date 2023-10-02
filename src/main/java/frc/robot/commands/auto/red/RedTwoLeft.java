package frc.robot.commands.auto.red;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.GrabSequenceCommand;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class RedTwoLeft extends BaseAutoRoute {

    public RedTwoLeft(RobotContainer rc) {
        super(Alliance.Red);

        addCommands(
                rc.waitForReady(),
                new ScoreFirstElementCommand(rc),

                rc.runTraj("red-get-element-left", true),
                new WaitCommand(1),
                rc.arm().setArmNormalSpeedCommand(),
                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
                rc.arm().waitForState(ArmMode.STOWED),
                new WaitCommand(0.5),
                rc.turret().goToAngle(Math.toRadians(90)),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.dt().resetGyroCommand(Rotation2d.fromDegrees(180))
//                rc.cv().transitionCommand(VisionState.CONE_DETECTOR),
//                new ParallelCommandGroup(
//                        rc.runTraj("red-get-element-left", true),
//                        new SequentialCommandGroup(
//                                new WaitCommand(1),
//                                rc.arm().setArmNormalSpeedCommand(),
//                                rc.arm().transitionCommand(ArmMode.SEEKING_PRIMED),
//                                rc.arm().waitForState(ArmMode.PRIMED),
//                                new WaitCommand(0.5),
//                                rc.turret().goToAngle(Math.toRadians(90)),
//                                new WaitCommand(0.5),
//                                rc.arm().setArmSlowSpeedCommand(),
//                                rc.arm().openClaw(),
//                                rc.arm().transitionCommand(Arm.ArmMode.NEW_GROUND_INTERMEDIATE),
//                                new WaitCommand(1.5),
//                                rc.arm().transitionCommand(ArmMode.NEW_GROUND_PICKUP)
//                        )
//                ),
//                rc.dt().waitForState(DrivetrainState.IDLE),
//                new GrabSequenceCommand(rc),
//                rc.arm().transitionCommand(ArmMode.LOW_SCORE),
//                rc.turret().goToAngle(Math.toRadians(-90)),
//                rc.runTraj("red-score-element-left"),
//                new WaitCommand(2.5),
//                rc.arm().openClaw()
        );
    }
}
