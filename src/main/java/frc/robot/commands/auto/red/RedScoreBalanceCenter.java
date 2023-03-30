package frc.robot.commands.auto.red;



import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class RedScoreBalanceCenter extends SequentialCommandGroup {

    public RedScoreBalanceCenter(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),

                new ScoreFirstElementCommand(rc),

                new ParallelCommandGroup(
                        rc.dt().transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION),
                        new WaitCommand(1).andThen(
                                rc.dt().waitForState(Drivetrain.DrivetrainState.IDLE),
                                rc.arm().waitForState(Arm.ArmMode.STOWED),
                                rc.turret().goToAngle(Math.toRadians(90))
                        )
                )
        );
    }
}
