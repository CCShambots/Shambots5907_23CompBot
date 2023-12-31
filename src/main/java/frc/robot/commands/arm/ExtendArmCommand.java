package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.kinematics.ArmState;

public class ExtendArmCommand extends Command {

  private final Arm a;
  private final ArmState target;

  private boolean elevatorSet;

  public ExtendArmCommand(Arm a, ArmState target) {
    this.a = a;
    this.target = target;
  }

  @Override
  public void initialize() {
    elevatorSet = false;

    a.setWristTarget(target.getWristAngle());
    a.setShoulderTarget(target.getShoulderAngle());
  }

  @Override
  public void execute() {
    if (!elevatorSet && a.isElevatorSafe()) {
      a.setElevatorTarget(target.getElevatorExtension());

      elevatorSet = true;
    }
  }

  @Override
  public boolean isFinished() {
    return elevatorSet;
  }
}
