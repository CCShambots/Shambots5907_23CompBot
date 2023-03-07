package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsytems.Lights;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Lights l = new Lights();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {

    return null; 
  }
}
