package frc.robot.commands.auto.blue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.GrabSequenceCommand;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;

public class BlueThreeScoreLeft extends BaseAutoRoute {

  public BlueThreeScoreLeft(RobotContainer rc) {

    super(Alliance.Blue, Math.toRadians(90));

    addCommands(
        rc.waitForReady(),
        rc.arm().transitionCommand(ArmMode.LOW_SCORE),
        rc.arm().openClaw(),
        new ParallelCommandGroup(
            rc.runTraj("blue-first-score-left", true),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                rc.turret().goToAngle(Math.toRadians(-90)),
                rc.arm().setArmSlowSpeedCommand(),
                new WaitCommand(0.5),
                rc.arm().transitionCommand(ArmMode.NEW_GROUND_INTERMEDIATE),
                new WaitCommand(1.25),
                rc.arm().transitionCommand(Arm.ArmMode.NEW_GROUND_PICKUP),
                rc.arm().openClaw())),
        rc.dt().waitForState(DrivetrainState.IDLE),
        new GrabSequenceCommand(rc),
        rc.arm().transitionCommand(ArmMode.LOW_SCORE),
        rc.turret().goToAngle(Math.toRadians(90)),
        rc.runTraj("blue-second-score-left"),
        new WaitCommand(2),
        rc.arm().openClaw(),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.runTraj("blue-get-second-element-left"),
        new WaitCommand(1),
        rc.turret().goToAngle(Math.toRadians(-90)),
        rc.arm().setArmSlowSpeedCommand(),
        rc.arm().transitionCommand(ArmMode.NEW_GROUND_INTERMEDIATE),
        new WaitCommand(1.5),
        rc.arm().transitionCommand(Arm.ArmMode.NEW_GROUND_PICKUP),
        rc.arm().openClaw(),
        rc.dt().waitForState(DrivetrainState.IDLE),
        new GrabSequenceCommand(rc),
        rc.arm().transitionCommand(ArmMode.LOW_SCORE),
        rc.runTraj("blue-third-score-left"),
        new WaitCommand(0.5),
        rc.turret().goToAngle(Math.toRadians(90)),
        new WaitCommand(1.95),
        rc.arm().openClaw()
        // rc.turret().setStartAngle(Math.toRadians(-90))
        );
  }
}
