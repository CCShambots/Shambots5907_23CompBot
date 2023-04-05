package frc.robot.commands.auto.red;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BonkShot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Turret.TurretState;

public class RedBalanceCenter extends SequentialCommandGroup {

    public RedBalanceCenter(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                rc.dt().resetGyroCommand(new Rotation2d()),

                new BonkShot(rc),

                new ParallelCommandGroup(
                        rc.dt().transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION),
                        new WaitCommand(1).andThen(
                                rc.arm().setArmNormalSpeedCommand(),
                                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED),
                                rc.arm().waitForState(ArmMode.STOWED),
                                new WaitCommand(2),
                                rc.turret().goToAngle(Math.toRadians(90))
                        )
                ), 
                rc.dt().waitForFlag(DrivetrainState.HIT_ZERO),
                rc.dt().waitForState(DrivetrainState.IDLE),
                rc.dt().transitionCommand(DrivetrainState.DOCKING),
                rc.dt().waitForState(DrivetrainState.BALANCING),
                rc.turret().transitionCommand(TurretState.IDLE)
        );
    }
}
