package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.subsystems.Drivetrain;


public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  Drivetrain dt;

  public RobotContainer() {
    dt = new Drivetrain(controller);
    dt.setFieldRelative(true);

    SubsystemManagerFactory.getInstance().registerSubsystem(dt);

    configureBindings();
  }

  private void configureBindings() {
    controller.x().onTrue(new InstantCommand(() -> dt.requestTransition(Drivetrain.DrivetrainState.XShape)));
    controller.b().onTrue(new InstantCommand(() -> dt.requestTransition(Drivetrain.DrivetrainState.TeleopDrive)));
    controller.a().onTrue(new InstantCommand(() -> dt.requestTransition(Drivetrain.DrivetrainState.Idle)));
    controller.y().onTrue(new InstantCommand(() -> dt.resetGyro(Rotation2d.fromDegrees(0))));
  }

  public Command getAutonomousCommand() {

    return null; 
  }
}
