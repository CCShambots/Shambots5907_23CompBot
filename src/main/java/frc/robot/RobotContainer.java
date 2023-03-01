package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.AutonomousLoader;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.subsystems.BaseVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.Vision.BASE_LIMELIGHT_POSE;
import static frc.robot.RobotContainer.AutoRoutes.*;

public class RobotContainer {

  //Declare HIDs
  private final CommandFlightStick leftStick = new CommandFlightStick(0);
  private final CommandFlightStick rightStick = new CommandFlightStick(1);

  //Declare subsystems
  private final BaseVision baseVision;
  private final Drivetrain dt;

  //Declare autonomous loader
  private final AutonomousLoader<AutoRoutes> autoLoader;

  private final Map<String, PathPlannerTrajectory> trajectories = new HashMap<>();

  public RobotContainer() {

    baseVision = new BaseVision(BASE_LIMELIGHT_POSE, () -> new Rotation2d()); //TODO: Give turret information to the vision subsystem

    dt = new Drivetrain(
          () -> -leftStick.getX(),
          () -> -leftStick.getY(),
          () -> -rightStick.getRawAxis(0),
          baseVision.getLLPoseSupplier(),
          baseVision.getLLHasTargetSupplier()
    );

    //Load the trajectories into the hashmap
    loadPaths("test");

    SubsystemManagerFactory.getInstance().registerSubsystem(dt);

    autoLoader = instantiateAutoLoader();


    configureBindings();
  }

  private AutonomousLoader<AutoRoutes> instantiateAutoLoader() {
    final AutonomousLoader<AutoRoutes> autoLoader;

    //Put new auto routes here
    autoLoader = new AutonomousLoader<>(Map.of(
            TEST, new InstantCommand()
    ));

    SmartDashboard.putData("auto-route", autoLoader.getSendableChooser());

    return autoLoader;
  }

  private void configureBindings() {
    rightStick.trigger().onTrue(new InstantCommand(() -> dt.requestTransition(DrivetrainState.X_SHAPE)));
    rightStick.trigger().onFalse(new InstantCommand(() -> dt.requestTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE)));

    leftStick.trigger().onTrue(new InstantCommand(dt::resetGyro));
  }

  public Command getAutonomousCommand() {
    return autoLoader.getCurrentSelection();
  }

  /**
   * Load a sequence of paths directly into the map of trajectories.
   * Calling this method again with the same names will regenerate the trajectory and replace the old instance of the trajectory
   * @param reversed whether the trajectories should be loaded as reversed
   * @param names the names of the trajectories to load
   */
  public void loadPaths(boolean reversed, String... names) {
    for (String n : names) {
      trajectories.put(n, PathPlanner.loadPath(n, Constants.SwerveDrivetrain.MAX_LINEAR_SPEED,
              Constants.SwerveDrivetrain.MAX_LINEAR_ACCELERATION, reversed));
    }
  }

  /**
   * Load a sequence of paths directly into the map of trajectories
   * @param names the names of the trajectories to load
   */
  public void loadPaths(String... names) {
    loadPaths(false, names);
  }

  public enum AutoRoutes {
    TEST
  }
}
