package frc.robot.commands.auto.red;



import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.ScoreFirstElementCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class RedBalanceCenter extends SequentialCommandGroup {

    public RedBalanceCenter(RobotContainer rc) {
        addCommands(
                rc.waitForReady(),
                
                //Score
                new ScoreFirstElementCommand(rc),
                
                //Get the arm safe to stow
                new WaitCommand(1),

                //Start docking the first time
                rc.dt().setPositiveDockDirectionCommand(false),
                rc.dt().transitionCommand(DrivetrainState.DOCKING),
                rc.arm().transitionCommand(Arm.ArmMode.SEEKING_STOWED),
                rc.dt().waitForState(DrivetrainState.BALANCING),
                rc.dt().setFlagCommand(DrivetrainState.DONT_BALANCE),
                
                //Wait for the bot to finish balancing
                rc.dt().waitForState(Drivetrain.DrivetrainState.IDLE),
                rc.arm().waitForState(Arm.ArmMode.STOWED),
                rc.turret().goToAngle(Math.toRadians(90)),

                //Mount the first time
                rc.dt().setPositiveDockDirectionCommand(false),
                rc.dt().transitionCommand(DrivetrainState.DOCKING),
                rc.dt().waitForState(DrivetrainState.BALANCING),
                rc.dt().setFlagCommand(DrivetrainState.DONT_BALANCE),
                new ParallelRaceGroup(
                        new WaitCommand(2),
                        rc.dt().waitForState(DrivetrainState.IDLE)
                ),
                rc.dt().transitionCommand(DrivetrainState.IDLE),

                //Dismount the charge station
                new PrintCommand("STARTED DRIVING"),
                new InstantCommand(() -> rc.dt().drive(
                                        new ChassisSpeeds(-0.25, 0, 0)
                                        , false)),
                new WaitCommand(1),
                new PrintCommand("FINISHED DRIVING"),
                new InstantCommand(() -> rc.dt().drive(new ChassisSpeeds(0, 0, 0), false)),
                new PrintCommand("DRIVING DONE"),


                //Actually balance
                rc.dt().setPositiveDockDirectionCommand(true),
                rc.dt().transitionCommand(DrivetrainState.DOCKING)
        );
    }
}
