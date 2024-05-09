package frc.robot.commands.auto.red;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.turret.Turret.TurretState;

public class RedPickupBalanceCenter extends BaseAutoRoute {

  public RedPickupBalanceCenter(RobotContainer rc) {

    super(Alliance.Red);

    addCommands(
        rc.waitForReady(),
        rc.dt().resetGyroCommand(new Rotation2d()),
        new ScoreFirstElementCommand(rc),
        rc.arm().openClaw(),
        new ParallelCommandGroup(
            rc.dt().transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION),
            new WaitCommand(1)
                .andThen(
                    rc.arm().setArmNormalSpeedCommand(),
                    rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED),
                    rc.arm().waitForState(ArmMode.STOWED),
                    new WaitCommand(2),
                    rc.turret().goToAngle(Math.toRadians(120)))),
        rc.dt().waitForFlag(DrivetrainState.HIT_ZERO),
        rc.arm().setArmSlowSpeedCommand(),
        rc.arm().transitionCommand(ArmMode.SEEKING_PICKUP_GROUND),
        new WaitCommand(1.5),
        rc.dt().waitForState(DrivetrainState.IDLE),
        rc.turret().transitionCommand(TurretState.INTAKING),
        new WaitCommand(1.5),
        new InstantCommand(
            () ->
                rc.dt()
                    .drive(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            -0.5, -0.25, 0, rc.dt().getCurrentAngle()))),
        new WaitCommand(0.75),
        new InstantCommand(() -> rc.dt().stopModules()),
        rc.arm().closeClaw(),
        new WaitCommand(0.5),
        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        rc.dt().transitionCommand(DrivetrainState.DOCKING),
        rc.arm().waitForState(ArmMode.STOWED),
        rc.dt().waitForState(DrivetrainState.BALANCING),
        rc.turret().transitionCommand(TurretState.IDLE),
        rc.turret().setStartAngle(Math.toRadians(-90)),
        rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
        rc.dt().waitForState(DrivetrainState.IDLE));
  }
}
