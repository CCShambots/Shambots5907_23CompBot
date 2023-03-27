package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShamLib.AutonomousLoader;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.commands.auto.blue.BlueScoreBalanceCenter;
import frc.robot.commands.auto.blue.BlueScoreBalanceLeft;
import frc.robot.commands.auto.blue.BlueScoreLeft;
import frc.robot.commands.auto.blue.BlueScoreRight;
import frc.robot.commands.auto.red.*;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.subsystems.Turret.TurretState;
import frc.robot.util.grid.GridElement;

import java.util.HashMap;
import java.util.Map;

import org.opencv.photo.TonemapReinhard;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.Vision.BASE_LIMELIGHT_POSE;
import static frc.robot.Constants.alliance;
import static frc.robot.Constants.gridInterface;
import static frc.robot.RobotContainer.AutoRoutes.*;
import static frc.robot.subsystems.Drivetrain.SpeedMode.NORMAL;
import static frc.robot.subsystems.Drivetrain.SpeedMode.TURBO;

public class RobotContainer extends StateMachine<RobotContainer.State> {

  //Declare HIDs
  private final CommandFlightStick leftStick = new CommandFlightStick(0);
  private final CommandFlightStick rightStick = new CommandFlightStick(1);
  private final CommandXboxController operatorCont = new CommandXboxController(2);

  //Declare subsystems
  private final BaseVision baseVision;
  private final ClawVision clawVision;
  private final Drivetrain drivetrain;
  private final Arm arm;
  private final Lights lights;
  private final Turret turret;

  //Declare autonomous loader
  private final AutonomousLoader<AutoRoutes> autoLoader;

  private final HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();
  

  public RobotContainer(EventLoop checkModulesLoop) {
    super("Robot", State.UNDETERMINED, State.class);

    arm = new Arm();
    lights = new Lights();
    clawVision = new ClawVision();

    turret = new Turret(
            operatorCont.pov(180),
            operatorCont.pov(0),
            clawVision::hasTarget,
            () -> clawVision.getGameElementOffset().getRadians(),
            operatorCont.pov(270),
            operatorCont.pov(90)
    );

    baseVision = new BaseVision(BASE_LIMELIGHT_POSE, () -> new Rotation2d(turret.getTurretAngle()));

    drivetrain = new Drivetrain(
          () -> -leftStick.getY(),
          () -> -leftStick.getX(),
          () -> -rightStick.getRawAxis(0),
          baseVision.getPoseSupplier(),
          baseVision.getLLHasTargetSupplier()
    );

    drivetrain.registerMisalignedSwerveTriggers(checkModulesLoop);


    //Load the trajectories into the hashmap
    loadPaths(
        "red-pickup-right",
        "red-dock-right",
        "red-dock-center",
        "red-score-left",
        "blue-dock-left",
        "blue-pickup-left",
        "blue-dock-center",
        "blue-score-right",
        "red-get-element-right",
        "red-go-score-right",
        "red-balance-right"
    );

    autoLoader = instantiateAutoLoader();

    initializeDriveTab();

    addChildSubsystem(drivetrain);
    addChildSubsystem(arm);
    addChildSubsystem(lights);
    addChildSubsystem(clawVision);
    addChildSubsystem(turret);

    defineTransitions();
    defineStateCommands();

    configureBindings();
  }

  private void defineTransitions() {
    addOmniTransition(State.DISABLED, new ParallelCommandGroup(
            drivetrain.transitionCommand(DrivetrainState.X_SHAPE),
            arm.transitionCommand(ArmMode.SOFT_STOP),
            turret.transitionCommand(Turret.TurretState.SOFT_STOP),
            lights.transitionCommand(LightState.SOFT_STOP)
    ));

    addOmniTransition(State.BRAKE, new ParallelCommandGroup(
            drivetrain.transitionCommand(DrivetrainState.X_SHAPE),
            arm.transitionCommand(ArmMode.SEEKING_STOWED),
            lights.transitionCommand(LightState.IDLE),
            turret.transitionCommand(Turret.TurretState.IDLE)
    ));

    addTransition(State.DISABLED, State.AUTONOMOUS);

    addOmniTransition(State.TRAVELING, new ParallelCommandGroup(
            arm.transitionCommand(ArmMode.SEEKING_STOWED),
            drivetrain.transitionCommand(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE),
            turret.transitionCommand(Turret.TurretState.CARDINALS)
    ));

    addTransition(State.TRAVELING, State.INTAKING, new ParallelCommandGroup(
            arm.transitionCommand(ArmMode.PICKUP_DOUBLE),
            turret.transitionCommand(Turret.TurretState.INTAKING)
    ));

    addTransition(State.TRAVELING, State.SCORING, new ParallelCommandGroup(
            lights.transitionCommand(LightState.SCORING),
            new InstantCommand(() -> arm.requestTransition(getNextScoringMode())),
            turret.transitionCommand(Turret.TurretState.SCORING)
    ));

    //TODO: REMOVE ALL TESTING STUFF
    addOmniTransition(State.TESTING, new ParallelCommandGroup(
            lights.transitionCommand(LightState.IDLE),
            drivetrain.transitionCommand(DrivetrainState.IDLE),
            turret.transitionCommand(TurretState.IDLE),
            arm.transitionCommand(ArmMode.TESTING)
    ));
  }

  private void defineStateCommands() {
    registerStateCommand(State.TRAVELING, new RunCommand(() -> {
      LightState correctState = gridInterface.getNextElement().isCube() ? LightState.CUBE : LightState.CONE;

      if (lights.getState() != correctState) {
        lights.requestTransition(correctState);
      }
    }));

    //perhaps dt drive over when scoring
  }

  private ArmMode getNextScoringMode() {
    GridElement e = gridInterface.getNextElement();

    switch (e.getRow()) {
      case 0:
        return ArmMode.LOW_SCORE;
      case 1:
        return ArmMode.MID_SCORE;
      case 2:
        return e.isCube() ? ArmMode.HIGH_CUBE : ArmMode.SEEKING_HIGH;
      default:
        return ArmMode.SEEKING_STOWED;
    }
  }

  private void initializeDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Auto Route", autoLoader.getSendableChooser()).withPosition(4, 0).withSize(2, 2);
    driveTab.addString("ALLIANCE", () -> alliance.name()).withPosition(0, 0).withSize(2, 2);
    driveTab.add("SWITCH ALLIANCE", switchAlliance()).withPosition(7,2).withSize(2, 2);
    driveTab.add("SYNC ALLIANCE", syncAlliance()).withPosition(7,0).withSize(2, 2);
    driveTab.addBoolean("Matching Auto", () -> autoLoader.getSendableChooser().getSelected().toString().toLowerCase().indexOf(alliance.name().toLowerCase()) != -1)
            .withPosition(4, 2).withSize(2, 2);
  }

  private AutonomousLoader<AutoRoutes> instantiateAutoLoader() {
    final AutonomousLoader<AutoRoutes> autoLoader;

    //Put new auto routes here
    autoLoader = new AutonomousLoader<>(Map.of(
      RED_SCORE_RIGHT, new RedScoreRight(this),
      RED_SCORE_BALANCE_RIGHT, new RedScoreBalanceRight(this),
      RED_SCORE_BALANCE_CENTER, new RedScoreBalanceCenter(this),
      RED_SCORE_LEFT, new RedScoreLeft(this),
      RED_NEW_AUTO, new RedNewAuto(this),
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
              Constants.pullAllianceFromFMS(this);
              Constants.overrideAlliance = false;
            }
    );
  }

  private void configureBindings() {
    
    rightStick.trigger().onTrue(transitionCommand(State.BRAKE));
    rightStick.trigger().onFalse(transitionCommand(State.TRAVELING));

    leftStick.topBase().onTrue(new InstantCommand(drivetrain::resetGyro));

    // leftStick.trigger().onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(TURBO)))
    //                 .onFalse(new InstantCommand(() -> drivetrain.setSpeedMode(NORMAL)));

    // rightStick.topBase().onTrue(drivetrain.transitionCommand(DrivetrainState.DOCKING));

    // operatorCont.a().onTrue(transitionCommand(State.TRAVELING));
    // operatorCont.b().onTrue(/*handleManualRequest(INTAKING)*/ arm.transitionCommand(ArmMode.PICKUP_DOUBLE));
    // operatorCont.x().onTrue(/*handleManualRequest(SCORING)*/ arm.transitionCommand(ArmMode.MID_SCORE));

    //TODO: remove when done testing
    Constants.Testing.STOP = operatorCont.y();
    Constants.Testing.RAISE = operatorCont.pov(0);
    Constants.Testing.LOWER = operatorCont.pov(180);

    operatorCont.leftBumper().onTrue(arm.openClaw());
    operatorCont.rightBumper().onTrue(arm.closeClaw());

    // operatorCont.leftTrigger(0.8)
    //         .and(operatorCont.rightTrigger(0.8))
    //         .onTrue(transitionCommand(State.DISABLED));

    // operatorCont.pov(90).onTrue(new InstantCommand(this::handleManualTurretRequest));
    // operatorCont.pov(270).onTrue(new InstantCommand(this::handleManualTurretRequest));

    // operatorCont.button(9).onTrue(arm.transitionCommand(ArmMode.SEEKING_PICKUP_GROUND).alongWith(turret.transitionCommand(TurretState.INTAKING)));
    // operatorCont.button(10).onTrue(arm.transitionCommand(ArmMode.SEEKING_STOWED));
    // operatorCont.button(9).onTrue(arm.transitionCommand(ArmMode.SEEKING_PICKUP_GROUND).alongWith(turret.transitionCommand(TurretState.INTAKING)));
    // operatorCont.button(10).onTrue(arm.transitionCommand(ArmMode.SEEKING_STOWED));

    operatorCont.a().onTrue(arm.transitionCommand(ArmMode.SEEKING_STOWED));
    operatorCont.b().onTrue(arm.transitionCommand(ArmMode.SEEKING_HIGH));
    operatorCont.x().onTrue(arm.transitionCommand(ArmMode.MID_SCORE));
    operatorCont.y().onTrue(arm.transitionCommand(ArmMode.PICKUP_DOUBLE));

  }

  private void handleManualTurretRequest() {
    if (isFlag(State.MANUAL_CONTROL)) return;

    if (getState() == State.INTAKING) handleManualRequest(State.INTAKING, Turret.TurretState.INTAKING);
    else if (getState() == State.SCORING) handleManualRequest(State.SCORING, Turret.TurretState.SCORING);
  }

  private void handleManualRequest(State s, Turret.TurretState ts) {
    if (!(s == State.SCORING || s == State.INTAKING)) return;

    if (getState() == s) {
      if (isFlag(State.MANUAL_CONTROL)) {
        clearFlag(State.MANUAL_CONTROL);
        turret.requestTransition(ts);
      }
      else {
        setFlag(State.MANUAL_CONTROL);
        turret.requestTransition(Turret.TurretState.MANUAL_CONTROL);
      }
    }
    else {
      requestTransition(s);
    }
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
    return drivetrain.runTrajectory(traj, DrivetrainState.IDLE);
  }

  public Command runTraj(PathPlannerTrajectory traj, boolean resetPose) {
    return drivetrain.runTrajectory(traj, resetPose, DrivetrainState.IDLE);
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
    return drivetrain;
  }

  public Turret turret() {
    return turret;
  }

  //TODO: Remove
  public void updateTarget() {
    drivetrain.getField().getObject("target").setPose(new Pose2d(gridInterface.getNextElement().getLocation().toTranslation2d(), new Rotation2d()));
  }

  public Command waitForReady() {
    return new WaitUntilCommand(() ->
            drivetrain.getState() == DrivetrainState.IDLE &&
            arm.getState() == ArmMode.STOWED
    );
  }

  @Override
  protected void determineSelf() {
    setState(State.TRAVELING);
  }

  @Override
  protected void onDisable() {
    setState(State.DISABLED);
  }

  @Override
  protected void onTeleopStart() {
    // requestTransition(State.TRAVELING);
    //TODO: Sussy
    // new WaitCommand(134).andThen(transitionCommand(State.BRAKE)).schedule();
  }

  @Override
  protected void onAutonomousStart() {
    registerStateCommand(State.AUTONOMOUS, getAutonomousCommand());
    requestTransition(State.AUTONOMOUS);
  }

  public enum State {
    INTAKING, SCORING, BALANCING, DISABLED, AUTONOMOUS, UNDETERMINED, TRAVELING, BRAKE, TESTING,

    MANUAL_CONTROL
  }


  public enum AutoRoutes {
    NOTHING,
    RED_SCORE_RIGHT, RED_SCORE_BALANCE_RIGHT, RED_SCORE_BALANCE_CENTER, RED_SCORE_LEFT, RED_NEW_AUTO,
    BLUE_SCORE_LEFT, BLUE_SCORE_BALANCE_LEFT, BLUE_SCORE_BALANCE_CENTER, BLUE_SCORE_RIGHT
  }
}
