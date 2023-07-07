package frc.robot.commands.auto.red;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.GrabSequenceCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class RedThreeScoreRight extends BaseAutoRoute {

    public RedThreeScoreRight(RobotContainer rc) {

        super(Alliance.Red, Math.toRadians(-90));

        addCommands(
                rc.waitForReady(),
                rc.arm().transitionCommand(ArmMode.LOW_SCORE),
                rc.arm().openClaw(),
                
                rc.cv().transitionCommand(VisionState.CONE_DETECTOR),
                new ParallelCommandGroup(
                        rc.runTraj("red-first-score-right", true),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                rc.turret().goToAngle(Math.toRadians(90)),
                                rc.arm().setArmSlowSpeedCommand(),
                                new WaitCommand(0.5),
                                rc.arm().transitionCommand(ArmMode.NEW_GROUND_INTERMEDIATE),
                                new WaitCommand(1.25),
                                rc.arm().transitionCommand(Arm.ArmMode.NEW_GROUND_PICKUP),
                                rc.arm().openClaw()
                        )
                ),
                rc.dt().waitForState(DrivetrainState.IDLE),
                new GrabSequenceCommand(rc),
                rc.arm().transitionCommand(ArmMode.LOW_SCORE),
                rc.turret().goToAngle(Math.toRadians(-90)),
                rc.runTraj("red-second-score-right"),
                new WaitCommand(2),
                rc.arm().openClaw(),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.runTraj("red-get-second-element-right"),
                new WaitCommand(1),
                rc.turret().goToAngle(Math.toRadians(90)),
                rc.arm().setArmSlowSpeedCommand(),
                rc.arm().transitionCommand(ArmMode.NEW_GROUND_INTERMEDIATE),
                new WaitCommand(1),
                rc.arm().transitionCommand(Arm.ArmMode.NEW_GROUND_PICKUP),
                rc.arm().openClaw(),
                rc.dt().waitForState(DrivetrainState.IDLE),
                new GrabSequenceCommand(rc),
                rc.arm().transitionCommand(ArmMode.LOW_SCORE),
                rc.runTraj("red-third-score-right"),
                new WaitCommand(0.5),
                
                rc.turret().goToAngle(Math.toRadians(-90)),
                new WaitCommand(1.95),
                rc.arm().openClaw(),

                new InstantCommand(() -> {
                        new WaitCommand(4)
                        .andThen(new WhileDisabledInstantCommand(() -> rc.dt().resetGyro(new Rotation2d(Math.toRadians(180))))).schedule();
                }),
                rc.turret().setStartAngle(Math.toRadians(90))
        );
    }
}
