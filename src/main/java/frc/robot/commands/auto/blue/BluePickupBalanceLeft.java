package frc.robot.commands.auto.blue;

import static java.lang.Math.toRadians;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.turret.Turret.TurretState;

public class BluePickupBalanceLeft extends BaseAutoRoute {

  public BluePickupBalanceLeft(RobotContainer rc) {

    super(Alliance.Blue);

    addCommands(
        rc.waitForReady(),
        new ScoreFirstElementCommand(rc),
        rc.cv().transitionCommand(VisionState.CONE_DETECTOR),
        new ParallelCommandGroup(
            rc.runTraj("blue-get-element-left", true),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                rc.turret().goToAngle(toRadians(-90)),
                rc.arm().setArmSlowSpeedCommand(),
                rc.arm().transitionCommand(ArmMode.SEEKING_PICKUP_GROUND),
                new WaitCommand(2),
                rc.turret().transitionCommand(TurretState.INTAKING),
                rc.arm().openClaw())),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.arm().closeClaw(),
        new WaitCommand(0.5),
        rc.arm().setArmNormalSpeedCommand(),
        rc.turret().transitionCommand(TurretState.IDLE),
        rc.turret().goToAngle(toRadians(-90)),
        new ParallelCommandGroup(
            rc.arm().transitionCommand(ArmMode.SEEKING_PRIMED), rc.runTraj("blue-go-balance-left")),
        rc.arm().setArmSlowSpeedCommand(),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.dt().setPositiveDockDirectionCommand(false),
        rc.dt().transitionCommand(DrivetrainState.DOCKING),
        rc.dt().waitForState(DrivetrainState.BALANCING),
        rc.turret().goToAngle(toRadians(90)),
        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        rc.dt().waitForState(DrivetrainState.X_SHAPE),
        rc.arm().setArmNormalSpeedCommand());
  }
}
