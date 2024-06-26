package frc.robot.commands.auto.blue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.turret.Turret.TurretState;

public class BluePickupLeft extends BaseAutoRoute {

  public BluePickupLeft(RobotContainer rc) {

    super(Alliance.Blue);

    addCommands(
        rc.waitForReady(),
        new ScoreFirstElementCommand(rc),
        new ParallelCommandGroup(
            rc.runTraj("blue-get-element-left", true),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                rc.turret().goToAngle(Math.toRadians(-90)),
                new WaitCommand(1),
                rc.arm().setArmSlowSpeedCommand(),
                rc.arm().transitionCommand(ArmMode.SEEKING_PICKUP_GROUND),
                new WaitCommand(1.5),
                rc.turret().transitionCommand(TurretState.INTAKING),
                rc.arm().openClaw())),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.arm().closeClaw(),
        new WaitCommand(0.5),
        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        rc.turret().transitionCommand(TurretState.IDLE),
        rc.turret().goToAngle(Math.toRadians(90)),
        rc.arm().setArmNormalSpeedCommand());
  }
}
