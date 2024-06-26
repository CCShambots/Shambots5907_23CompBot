package frc.robot.commands.auto.red;

import static java.lang.Math.toRadians;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretState;

public class RedPickupBalanceRight extends BaseAutoRoute {

  public RedPickupBalanceRight(RobotContainer rc) {

    super(Alliance.Red);

    addCommands(
        rc.waitForReady(),
        new ScoreFirstElementCommand(rc),
        new ParallelCommandGroup(
            rc.runTraj("red-get-element-right", true),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                rc.turret().goToAngle(toRadians(90)),
                rc.arm().setArmSlowSpeedCommand(),
                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_PICKUP_GROUND),
                new WaitCommand(1.5),
                rc.turret().transitionCommand(Turret.TurretState.INTAKING),
                rc.arm().openClaw())),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.arm().closeClaw(),
        new WaitCommand(0.5),
        rc.arm().setArmNormalSpeedCommand(),
        rc.turret().transitionCommand(TurretState.IDLE),
        rc.turret().goToAngle(toRadians(90)),
        new ParallelCommandGroup(
            rc.arm().transitionCommand(ArmMode.SEEKING_PRIMED), rc.runTraj("red-go-balance-right")),
        rc.arm().setArmSlowSpeedCommand(),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.dt().transitionCommand(DrivetrainState.DOCKING),
        rc.dt().waitForState(DrivetrainState.BALANCING),
        rc.turret().goToAngle(toRadians(-90)),
        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        rc.dt().waitForState(DrivetrainState.X_SHAPE),
        rc.arm().setArmNormalSpeedCommand());
  }
}
