package frc.robot.commands.auto.blue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Turret.TurretState;

public class BlueBalanceCenter extends BaseAutoRoute {

    public BlueBalanceCenter(RobotContainer rc) {

        super(Alliance.Blue, Math.toRadians(90));

        addCommands(
                rc.waitForReady(),
                rc.dt().resetGyroCommand(new Rotation2d()),

                new ScoreFirstElementCommand(rc),

                rc.dt().setPositiveDockDirectionCommand(false),
                new ParallelCommandGroup(
                        rc.dt().transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION),
                        new WaitCommand(1).andThen(
                                rc.arm().setArmNormalSpeedCommand(),
                                rc.arm().transitionCommand(ArmMode.SEEKING_STOWED),
                                rc.arm().waitForState(ArmMode.STOWED),
                                new WaitCommand(2),
                                rc.turret().goToAngle(Math.toRadians(-90))
                        )
                ), 
                // rc.dt().waitForFlag(DrivetrainState.HIT_ZERO),
                rc.dt().waitForState(DrivetrainState.IDLE),
                new WaitCommand(1),
                rc.dt().transitionCommand(DrivetrainState.DOCKING),
                rc.dt().waitForState(DrivetrainState.BALANCING),
                rc.turret().transitionCommand(TurretState.IDLE)
        );
    }
}
