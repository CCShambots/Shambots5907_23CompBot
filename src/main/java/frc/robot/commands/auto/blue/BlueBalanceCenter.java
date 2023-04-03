package frc.robot.commands.auto.blue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class BlueBalanceCenter extends SequentialCommandGroup {

    public BlueBalanceCenter(RobotContainer rc) {
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
                )
        );
    }
}
