package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.subsystems.Drivetrain;


public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);

  Joystick rightHandCont = new Joystick(1);
  Drivetrain dt;

  public RobotContainer() {
    dt = new Drivetrain(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -rightHandCont.getRawAxis(0)
    );
    dt.setFieldRelative(true);

    SubsystemManagerFactory.getInstance().registerSubsystem(dt);

    configureBindings();
  }

  private void configureBindings() {
    controller.x().onTrue(new InstantCommand(() -> dt.requestTransition(Drivetrain.DrivetrainState.XShape)));
    controller.b().onTrue(new InstantCommand(() -> dt.requestTransition(Drivetrain.DrivetrainState.FieldOrientedTeleopDrive)));
    // controller.a().onTrue(new InstantCommand(() -> dt.requestTransition(Drivetrain.DrivetrainState.Idle)));
    controller.a().onTrue(new InstantCommand(() -> dt.resetGyro()));

  }

  public Command getAutonomousCommand() {

    return null; 
  }
}
