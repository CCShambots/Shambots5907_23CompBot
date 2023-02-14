package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.subsystems.Claw;


public class RobotContainer {
  private final CommandXboxController m_driverController =
          new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  public final Claw claw;

  public RobotContainer() {
    claw = new Claw();


    SubsystemManagerFactory.getInstance().registerSubsystem(claw);
    configureBindings();
  }

  private void configureBindings() {
    System.out.println("a");
    m_driverController.a().onTrue(new InstantCommand(() -> claw.requestTransition(Claw.State.OPENED)));
    m_driverController.a().onFalse(new InstantCommand(() -> claw.requestTransition(Claw.State.CLOSED)));
  }

  public Command getAutonomousCommand() {

    return null; 
  }
}
