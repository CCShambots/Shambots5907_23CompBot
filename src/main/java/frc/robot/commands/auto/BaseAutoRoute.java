package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class BaseAutoRoute extends SequentialCommandGroup {

  public final double startAngle;
  public final Alliance alliance;

  public BaseAutoRoute(Alliance alliance, double startAngle, Command... commands) {
    this.startAngle = startAngle;
    this.alliance = alliance;

    addCommands(commands);
  }

  public BaseAutoRoute(Alliance alliance, Command... commands) {
    this(alliance, Constants.Turret.getTurretStartAngleFromAlliance(), commands);
  }

  public double getStartAngle() {
    return startAngle;
  }

  public Alliance getAlliance() {
    return alliance;
  }
}
