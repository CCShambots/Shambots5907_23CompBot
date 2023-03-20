package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.AutonomousLoader;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.commands.auto.blue.BlueScoreBalanceCenter;
import frc.robot.commands.auto.blue.BlueScoreBalanceLeft;
import frc.robot.commands.auto.blue.BlueScoreLeft;
import frc.robot.commands.auto.blue.BlueScoreRight;
import frc.robot.commands.auto.red.RedScoreBalanceCenter;
import frc.robot.commands.auto.red.RedScoreBalanceRight;
import frc.robot.commands.auto.red.RedScoreLeft;
import frc.robot.commands.auto.red.RedScoreRight;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BaseVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Lights.LightState;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.SwerveDrivetrain.MIN_TURBO_SPEED;
import static frc.robot.Constants.Vision.BASE_LIMELIGHT_POSE;
import static frc.robot.Constants.alliance;
import static frc.robot.RobotContainer.AutoRoutes.*;
import static frc.robot.subsystems.Drivetrain.SpeedMode.NORMAL;
import static frc.robot.subsystems.Drivetrain.SpeedMode.TURBO;

public class RobotContainer {

  //Declare HIDs
  private final CommandFlightStick leftStick = new CommandFlightStick(0);
  private final CommandFlightStick rightStick = new CommandFlightStick(1);
  private final CommandXboxController operatorCont = new CommandXboxController(2);

  //Declare subsystems
  private final BaseVision baseVision;
  private final Drivetrain dt;
  private final Arm arm;

  //Declare autonomous loader
  private final AutonomousLoader<AutoRoutes> autoLoader;
  
  private final HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();

  private final Lights l;

  public RobotContainer() {

    baseVision = new BaseVision(BASE_LIMELIGHT_POSE, () -> new Rotation2d()); //TODO: Give turret information to the vision subsystem

    dt = new Drivetrain(
          () -> -leftStick.getY(),
          () -> -leftStick.getX(),
          () -> -rightStick.getRawAxis(0),
          baseVision.getLLPoseSupplier(),
          baseVision.getLLHasTargetSupplier()
    );

    this.arm = new Arm();
    this.l = new Lights();

    //Load the trajectories into the hashmap
    loadPaths("red-pickup-right", "red-dock-right", "red-dock-center",
     "red-score-left", "blue-dock-left", "blue-pickup-left", "blue-dock-center", "blue-score-right");

    SubsystemManagerFactory.getInstance().registerSubsystem(dt);
    SubsystemManagerFactory.getInstance().registerSubsystem(arm, false);
    SubsystemManagerFactory.getInstance().registerSubsystem(l, false);

    autoLoader = instantiateAutoLoader();

    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Auto Route", autoLoader.getSendableChooser()).withPosition(4, 0).withSize(2, 2);
    driveTab.addString("ALLIANCE", () -> alliance.name()).withPosition(0, 0).withSize(2, 2);
    driveTab.add("SWITCH ALLIANCE", switchAlliance()).withPosition(7,2).withSize(2, 2);
    driveTab.add("SYNC ALLIANCE", syncAlliance()).withPosition(7,0).withSize(2, 2);
    driveTab.addBoolean("Matching Auto", () -> autoLoader.getSendableChooser().getSelected().toString().toLowerCase().indexOf(alliance.name().toLowerCase()) != -1)
    .withPosition(4, 2).withSize(2, 2);
    
    configureBindings();

  }

  private AutonomousLoader<AutoRoutes> instantiateAutoLoader() {
    final AutonomousLoader<AutoRoutes> autoLoader;

    //Put new auto routes here
    autoLoader = new AutonomousLoader<>(Map.of(
      RED_SCORE_RIGHT, new RedScoreRight(this),
      RED_SCORE_BALANCE_RIGHT, new RedScoreBalanceRight(this),
      RED_SCORE_BALANCE_CENTER, new RedScoreBalanceCenter(this),
      RED_SCORE_LEFT, new RedScoreLeft(this),
      BLUE_SCORE_LEFT, new BlueScoreLeft(this),
      BLUE_SCORE_BALANCE_LEFT, new BlueScoreBalanceLeft(this),
      BLUE_SCORE_BALANCE_CENTER, new BlueScoreBalanceCenter(this),
      BLUE_SCORE_RIGHT, new BlueScoreRight(this),
      NOTHING, new InstantCommand()
    ));

    return autoLoader;
  }

  private InstantCommand switchAlliance() {
    return new WhileDisabledInstantCommand(
            () -> {
              alliance = alliance == Red ? Blue : Red;
              Constants.overrideAlliance = true;
            }
    );
  }

  private InstantCommand syncAlliance() {
    return new WhileDisabledInstantCommand(
            () -> {
              Constants.pullAllianceFromFMS();
              Constants.overrideAlliance = false;
            }
    );
  }

  private void configureBindings() {
    rightStick.trigger().onTrue(new InstantCommand(() -> dt.requestTransition(DrivetrainState.X_SHAPE)));
    rightStick.trigger().onFalse(new InstantCommand(() -> dt.requestTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE)));

    leftStick.topBase().onTrue(new InstantCommand(dt::resetGyro));
    
    leftStick.trigger()
            .and(() -> arm.getState() == ArmMode.STOWED)
            .and(() -> dt.getTargetLinearSpeed() >= MIN_TURBO_SPEED)
            .onTrue(new InstantCommand(() -> dt.setSpeedMode(TURBO)));
    leftStick.trigger().onFalse(new InstantCommand(() -> dt.setSpeedMode(NORMAL)));
    new Trigger(() -> dt.getTargetLinearSpeed() < MIN_TURBO_SPEED)
            .or(() -> arm.getState() != ArmMode.STOWED)
            .onTrue(new InstantCommand(() -> dt.setSpeedMode(NORMAL)));

    // rightStick.trigger().onTrue(dt.calculateModuleDrive(leftStick.trigger(), rightStick.trigger(), () -> leftStick.topBase().getAsBoolean()));

    // rightStick.topBase().onTrue(new InstantCommand(() -> dt.setAllModules(new SwerveModuleState(0, new Rotation2d()))));

    operatorCont.leftBumper().onTrue(arm.openClaw());
    operatorCont.rightBumper().onTrue(arm.closeClaw());

    operatorCont.a().onTrue(new InstantCommand(() -> arm.requestTransition(ArmMode.SEEKING_STOWED)));
    operatorCont.b().onTrue(new InstantCommand(() -> arm.requestTransition(ArmMode.PICKUP_DOUBLE)));

    operatorCont.pov(0).onTrue(new InstantCommand(() -> arm.requestTransition(ArmMode.MID_SCORE)));
    operatorCont.pov(90).onTrue(new InstantCommand(() -> arm.requestTransition(ArmMode.SEEKING_HIGH)));
    operatorCont.pov(270).onTrue(new InstantCommand(() -> arm.requestTransition(ArmMode.LOW_SCORE)));
    operatorCont.pov(180).onTrue(new InstantCommand(() -> arm.requestTransition(ArmMode.SEEKING_PICKUP_GROUND)));

    operatorCont.leftTrigger(0.8)
      .and(() -> operatorCont.rightTrigger(0.8)
      .getAsBoolean()).onTrue(arm.transitionCommand(ArmMode.SOFT_STOP));

    operatorCont.leftStick().onTrue(l.transitionCommand(LightState.CUBE));
    operatorCont.rightStick().onTrue(l.transitionCommand(LightState.UPRIGHT_CONE));

    SmartDashboard.putData(new InstantCommand(() -> Constants.pullAllianceFromFMS()));
  }

  public Command getAutonomousCommand() {
    return autoLoader.getCurrentSelection();
  }

  public Runnable runArmControlLoops() {
    return arm.runControlLoops();
  }


  /**
   * Load a sequence of paths directly into the map of trajectories.
   * Calling this method again with the same names will regenerate the trajectory and replace the old instance of the trajectory
   * @param reversed whether the trajectories should be loaded as reversed
   * @param names the names of the trajectories to load
   */
  public void loadPaths(boolean reversed, String... names) {
    for (String n : names) {
      trajectories.put(n, PathPlanner.loadPath(n, Constants.SwerveDrivetrain.MAX_LINEAR_SPEED_AUTO,
              Constants.SwerveDrivetrain.MAX_LINEAR_ACCELERATION_AUTO, reversed));
    }
  }

  /**
   * Load a sequence of paths directly into the map of trajectories
   * @param names the names of the trajectories to load
   */
  public void loadPaths(String... names) {
    loadPaths(false, names);
  }

  public Map<String, PathPlannerTrajectory> paths() {
    return trajectories;
  }

  public Command runTraj(PathPlannerTrajectory traj) {
    return dt.runTrajectory(traj, DrivetrainState.IDLE);
  }

  public Command runTraj(PathPlannerTrajectory traj, boolean resetPose) {
    return dt.runTrajectory(traj, resetPose, DrivetrainState.IDLE);
  }

  public Command runTraj(String traj) {
    return runTraj(paths().get(traj));
  }

  public Command runTraj(String traj, boolean resetPose) {
    return runTraj(paths().get(traj), resetPose);
  }

  public Arm arm() {
    return arm;
  }

  public Drivetrain dt() {
    return dt;
  }

  public Command waitForReady() {
    return new WaitUntilCommand(() ->
            dt.getState() == DrivetrainState.IDLE &&
            arm.getState() == ArmMode.STOWED
    );
  }

  public enum AutoRoutes {
    NOTHING,
    RED_SCORE_RIGHT, RED_SCORE_BALANCE_RIGHT, RED_SCORE_BALANCE_CENTER, RED_SCORE_LEFT,
    BLUE_SCORE_LEFT, BLUE_SCORE_BALANCE_LEFT, BLUE_SCORE_BALANCE_CENTER, BLUE_SCORE_RIGHT
  }
}
