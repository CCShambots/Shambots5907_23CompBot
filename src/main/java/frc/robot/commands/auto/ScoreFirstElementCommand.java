package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm.ArmMode;

public class ScoreFirstElementCommand extends SequentialCommandGroup {

  public ScoreFirstElementCommand(RobotContainer rc) {
    addCommands(
        rc.arm().setArmSlowSpeedCommand(),
        rc.arm().enableClawProx(),
        rc.arm().transitionCommand(ArmMode.PRIMED),
        new WaitCommand(1),
        rc.arm().transitionCommand(ArmMode.HIGH_CUBE),
        new WaitCommand(0.7),
        rc.arm().openClaw());
  }
}
