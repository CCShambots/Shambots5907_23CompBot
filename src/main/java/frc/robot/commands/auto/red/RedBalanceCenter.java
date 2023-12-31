package frc.robot.commands.auto.red;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.turret.Turret.TurretState;

public class RedBalanceCenter extends BaseAutoRoute {

  public RedBalanceCenter(RobotContainer rc) {
    super(Alliance.Red, Math.toRadians(90));

    addCommands(
        rc.waitForReady(),
        rc.dt().resetGyroCommand(new Rotation2d(Math.toRadians(180))),
        new ScoreFirstElementCommand(rc),
        new ParallelCommandGroup(
            rc.dt().transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION),
            new WaitCommand(1)
                .andThen(
                    rc.arm().setArmNormalSpeedCommand(),
                    rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED),
                    rc.arm().waitForState(ArmMode.STOWED),
                    new WaitCommand(2),
                    rc.turret().goToAngle(Math.toRadians(-90)))),
        rc.dt().waitForState(DrivetrainState.IDLE),
        new WaitCommand(1),
        rc.dt().transitionCommand(DrivetrainState.DOCKING),
        rc.dt().waitForState(DrivetrainState.BALANCING),
        rc.turret().transitionCommand(TurretState.IDLE));
  }
}
