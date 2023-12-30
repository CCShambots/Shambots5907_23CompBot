package frc.robot.commands.auto.blue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Turret.TurretState;

public class BlueTwoScoreLeft extends BaseAutoRoute {

  public BlueTwoScoreLeft(RobotContainer rc) {

    super(Alliance.Blue, Math.toRadians(90));

    addCommands(
        rc.waitForReady(),
        new ScoreFirstElementCommand(rc),
        rc.cv().transitionCommand(VisionState.CONE_DETECTOR),
        new ParallelCommandGroup(
            rc.runTraj("blue-get-element-left", true),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                rc.turret().goToAngle(Math.toRadians(-90)),
                rc.arm().setArmSlowSpeedCommand(),
                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_PICKUP_GROUND),
                new WaitCommand(2),
                rc.turret().transitionCommand(TurretState.INTAKING),
                rc.arm().openClaw())),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.arm().closeClaw(),
        new WaitCommand(0.5),
        rc.arm().transitionCommand(ArmMode.SEEKING_HIGH),
        rc.turret().transitionCommand(TurretState.IDLE),
        rc.turret().goToAngle(Math.toRadians(90)),
        // rc.arm().setArmNormalSpeedCommand(),
        rc.runTraj("blue-go-score-left"),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.arm().openClaw(),
        new WaitCommand(0.5),
        rc.runTraj("blue-back-off-left"),
        new WaitCommand(1),
        rc.arm().setArmNormalSpeedCommand(),
        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        rc.turret().setStartAngle(Math.toRadians(-90)));
  }
}
