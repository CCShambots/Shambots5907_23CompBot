package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.AutonomousLoader;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.commands.auto.blue.BlueBalanceCenter;
import frc.robot.commands.auto.blue.BluePickupBalanceLeft;
import frc.robot.commands.auto.blue.BluePickupLeft;
import frc.robot.commands.auto.blue.BluePickupRight;
import frc.robot.commands.auto.red.*;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.subsystems.Turret.TurretState;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static frc.robot.Constants.ElementType.*;
import static frc.robot.Constants.SwerveDrivetrain.MIN_TURBO_SPEED;
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

  private final PowerDistribution pd = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private ArmMode currentScoreMode = ArmMode.SEEKING_HIGH;
  private Constants.ElementType nextElement = Cone;

  //Declare autonomous loader
  private final AutonomousLoader<AutoRoutes> autoLoader;

  private final HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<>();

  public RobotContainer(EventLoop checkModulesLoop) {
    super("Robot", State.UNDETERMINED, State.class);

    arm = new Arm();
    lights = new Lights();
    clawVision = new ClawVision(arm::getClawState);

    turret = new Turret(
            operatorCont.x(),
            operatorCont.y(),
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
        "red-go-score-right",
        "red-balance-right",
        "red-go-balance-right",
        "blue-dock-left",
        "blue-pickup-left",
        "blue-score-right"
    );

    loadPaths(1.25, 1, "red-back-off-right");
    loadPaths(1.25, 1, "red-get-element-right");
    loadPaths(2, 2, "red-go-balance-right");
    loadPaths(0.75, 0.75, "red-pickup-left");
    
    loadPaths(1.25, 1, "blue-get-element-left");
    loadPaths(2, 2, "blue-go-balance-left");
    loadPaths(0.75, 0.75, "blue-pickup-right");

    autoLoader = instantiateAutoLoader();

    initializeDriveTab();

    addChildSubsystem(drivetrain);
    addChildSubsystem(arm);
    addChildSubsystem(lights);
    addChildSubsystem(clawVision);
    addChildSubsystem(baseVision);
    addChildSubsystem(turret);

    // TODO: Remove
    // SmartDashboard.putData("drivetrain", drivetrain);
    // SmartDashboard.putData("field", drivetrain.getField());
    SmartDashboard.putData("arm", arm);
    SmartDashboard.putData("base vision", baseVision);
    SmartDashboard.putData("turret", turret);
    SmartDashboard.putData("claw", arm.claw());

    defineTransitions();
    defineStateCommands();

    configureBindings();

    operatorCont.getHID().setRumble(kBothRumble, 0);
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
            // arm.transitionCommand(ArmMode.SEEKING_STOWED),
            lights.transitionCommand(LightState.IDLE),
            turret.transitionCommand(Turret.TurretState.IDLE)
    ));

    addTransition(State.DISABLED, State.AUTONOMOUS);

    addOmniTransition(State.TRAVELING, new ParallelCommandGroup(
            // arm.transitionCommand(ArmMode.SEEKING_STOWED),
            drivetrain.transitionCommand(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE),
            turret.transitionCommand(Turret.TurretState.CARDINALS),
            clawVision.transitionCommand(ClawVision.VisionState.ELEMENT_TYPE)
    ));

    addTransition(State.TRAVELING, State.INTAKING, new ParallelCommandGroup(
            arm.transitionCommand(ArmMode.SEEKING_PICKUP_DOUBLE)
            // turret.transitionCommand(Turret.TurretState.INTAKING)
    ));

    addTransition(State.TRAVELING, State.SCORING, new ParallelCommandGroup(
            lights.transitionCommand(LightState.SCORING),
            new InstantCommand(() -> arm.requestTransition(currentScoreMode))
            // turret.transitionCommand(Turret.TurretState.SCORING)
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
      LightState correctState = lights.getStateFromElements(nextElement, clawVision.getCurrentElementType());
      
      if (lights.getState() != correctState && lights.canDisplayInfo()){
        lights.requestTransition(correctState);
      }
    }));

    registerStateCommand(State.INTAKING, new RunCommand(() -> {
      LightState correctState = lights.getStateFromElements(nextElement, clawVision.getCurrentElementType());

      if(correctState == LightState.CONE) correctState = LightState.INTAKE_CONE;
      else if(correctState == LightState.CUBE) correctState = LightState.INTAKE_CUBE;

      if (lights.getState() != correctState && lights.canDisplayInfo()) {
        lights.requestTransition(correctState);
      }
    }));

  }

  private void initializeDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Auto Route", autoLoader.getSendableChooser()).withPosition(4, 0).withSize(2, 2);
    driveTab.addString("ALLIANCE", () -> alliance.name()).withPosition(0, 0).withSize(2, 2);
    driveTab.add("SWITCH ALLIANCE", switchAlliance()).withPosition(7,2).withSize(2, 2);
    driveTab.add("SYNC ALLIANCE", syncAlliance()).withPosition(7,0).withSize(2, 2);
    driveTab.addBoolean("Matching Auto", () -> autoLoader.getSendableChooser().getSelected().toString().toLowerCase().contains(alliance.name().toLowerCase()))
            .withPosition(4, 2).withSize(2, 2);
  }

  private AutonomousLoader<AutoRoutes> instantiateAutoLoader() {
    final AutonomousLoader<AutoRoutes> autoLoader;

    //Put new auto routes here
    Map<AutoRoutes, Command> routes = new HashMap<>();

    //Red routes
    routes.putAll(Map.of(
      RED_PICKUP_RIGHT, new RedPickupRight(this),
      RED_BALANCE_CENTER, new RedBalanceCenter(this),
      RED_PICKUP_LEFT, new RedPickupLeft(this),
      RED_PICKUP_BALANCE_RIGHT, new RedPickupBalanceRight(this)
    ));

    //Blue routes
    routes.putAll(Map.of(
            BLUE_PICKUP_BALANCE_LEFT, new BluePickupBalanceLeft(this),
            BLUE_PICKUP_LEFT, new BluePickupLeft(this),
            BLUE_BALANCE_CENTER, new BlueBalanceCenter(this),
            BLUE_PICKUP_RIGHT, new BluePickupRight(this) 
      )
    );

    //Route to do nothing just in case everything has gone wrong
    routes.put(
            NOTHING, new InstantCommand()
    );

    autoLoader = new AutonomousLoader<>(routes);

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

//    dt().enableTeleopAutobalanceControls(leftStick, rightStick);

    rightStick.trigger().onTrue(transitionCommand(State.BRAKE));
    rightStick.trigger().onFalse(transitionCommand(State.TRAVELING));


    leftStick.topBase().onTrue(new InstantCommand(drivetrain::resetGyro));

    //Turbo logic
    leftStick.trigger()
            .and(() -> arm.getState() == ArmMode.STOWED)
            .and(() -> drivetrain.getTargetLinearSpeed() >= MIN_TURBO_SPEED)
            .onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(TURBO)));
    leftStick.trigger().onFalse(new InstantCommand(() -> drivetrain.setSpeedMode(NORMAL)));
    new Trigger(() -> drivetrain.getTargetLinearSpeed() < MIN_TURBO_SPEED)
            .or(() -> arm.getState() != ArmMode.STOWED)
            .onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(NORMAL)));

    operatorCont.a().onTrue(transitionCommand(State.TRAVELING).alongWith(arm.transitionCommand(ArmMode.SEEKING_STOWED)));
    operatorCont.b().onTrue(new InstantCommand(() -> handleManualRequest(State.INTAKING, TurretState.INTAKING)));

    //Scoring modes for the arm
    operatorCont.pov(270).onTrue(new InstantCommand(() -> {
      currentScoreMode = ArmMode.LOW_SCORE;
      handleManualRequest(State.SCORING, TurretState.SCORING);
    }));

    operatorCont.pov(0).onTrue(new InstantCommand(() -> {
      currentScoreMode = ArmMode.MID_SCORE;
      handleManualRequest(State.SCORING, TurretState.SCORING);
    }));

    operatorCont.pov(90).onTrue(new InstantCommand(() -> {
      currentScoreMode = getHighScoreModeFromVision();
      handleManualRequest(State.SCORING, TurretState.SCORING);
    }));

    //Setting the arm to primed position
    operatorCont.pov(180).onTrue(arm.transitionCommand(ArmMode.PRIMED));

    //Claw controls
    operatorCont.leftBumper().onTrue(arm.openClaw());
    operatorCont.rightBumper().onTrue(arm.closeClaw());

    new Trigger(() -> operatorCont.getLeftX() > 0.8)
            .or(() -> operatorCont.getRightX() > 0.8)
            .onTrue(arm.enableClawProx().alongWith(lights.transitionCommand(LightState.ENABLE_PROX)));

    new Trigger(() -> operatorCont.getLeftX() < -0.8)
            .or(() -> operatorCont.getRightX() < -0.8)
            .onTrue(arm.disableClawProx().alongWith(lights.transitionCommand(LightState.DISABLE_PROX)));

    operatorCont.leftStick().onTrue(new InstantCommand(() -> nextElement = Cone));

    operatorCont.rightStick().onTrue(new InstantCommand(() -> nextElement = Cube));

    operatorCont.leftTrigger(0.8)
            .and(operatorCont.rightTrigger(0.8))
            .onTrue(transitionCommand(State.DISABLED));

    operatorCont.pov(90).and(() -> getState() == State.SCORING).onTrue(new InstantCommand(this::handleManualTurretRequest));
    operatorCont.pov(270).and(() -> getState() == State.SCORING).onTrue(new InstantCommand(this::handleManualTurretRequest));

    new Trigger(this::lowVoltage)
            .debounce(2)
            .onTrue(new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 1)))
            .onFalse(new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 0)));

    leftStick.topLeft().onTrue(baseVision.transitionCommand(BaseVision.BaseVisionState.APRILTAG).alongWith(turret.transitionCommand(TurretState.CARDINALS)));
    leftStick.topRight().onTrue(
      baseVision.transitionCommand(BaseVision.BaseVisionState.RETROREFLECTIVE).alongWith(
        turret.transitionCommand(TurretState.LIMELIGHT_SCORING)
      ));
  }

  public ArmMode getHighScoreModeFromVision() {
    if(clawVision.getState() == ClawVision.VisionState.ELEMENT_TYPE
            && clawVision.getCurrentElementType() == Constants.ElementType.Cube) {
      return ArmMode.HIGH_CUBE;
    } else return ArmMode.SEEKING_HIGH;
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
        // turret.requestTransition(Turret.TurretState.MANUAL_CONTROL);
      }
    }
    else {
      requestTransition(s);
    }
  }

  public Command setTurretToIntake() {
    return new ParallelCommandGroup(
      turret.transitionCommand(TurretState.INTAKING),
      clawVision.transitionCommand(VisionState.CONE_DETECTOR)
    );
  }

  public void scheduleEndgameBuzz() {
    new WaitCommand(103.8).andThen(
            rumbleLoop(),     
            rumbleLoop(),     
            rumbleLoop()  
    ).schedule();
  }

  private Command rumbleLoop() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 1)),
            new WaitCommand(0.25),
            new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 0)),
            new WaitCommand(0.15)
    );
  }

  public Command getAutonomousCommand() {
    return autoLoader.getCurrentSelection();
  }

  public Runnable runArmControlLoops() {
    return arm.runControlLoops();
  }

  public boolean lowVoltage() {
    return pd.getVoltage() <= Constants.VOLTAGE_WARNING;
  }

  /**
   * Load a sequence of paths directly into the map of trajectories.
   * Calling this method again with the same names will regenerate the trajectory and replace the old instance of the trajectory
   * @param reversed whether the trajectories should be loaded as reversed
   * @param names the names of the trajectories to load
   */
  public void loadPaths(double maxSpeed, double maxAccel, boolean reversed, String... names) {
    for (String n : names) {
      trajectories.put(n, PathPlanner.loadPath(n, maxSpeed, maxAccel, reversed));
    }
  }

  /**
   * Load a sequence of paths directly into the map of trajectories
   * @param names the names of the trajectories to load
   */
  public void loadPaths(double maxSpeed, double maxAccel, String... names) {
    loadPaths(maxSpeed, maxAccel, false, names);
  }

  public void loadPaths(String... names) {
    loadPaths(Constants.SwerveDrivetrain.MAX_LINEAR_SPEED_AUTO, Constants.SwerveDrivetrain.MAX_LINEAR_ACCELERATION_AUTO, names);
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

  public BaseVision bv() {return baseVision;}
  public ClawVision cv() {return clawVision;}
  
  public Lights lights() {
    return lights;
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

    operatorCont.getHID().setRumble(kBothRumble, 0);
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TRAVELING);
    new WaitCommand(134).andThen(new ConditionalCommand(transitionCommand(State.BRAKE), new InstantCommand(), () -> drivetrain.getTargetLinearSpeed() < 0.5)).schedule();
  }

  @Override
  protected void onAutonomousStart() {
    registerStateCommand(State.AUTONOMOUS, getAutonomousCommand());
    requestTransition(State.AUTONOMOUS);
  }

  public enum State {
    INTAKING, SCORING, BALANCING, DISABLED, AUTONOMOUS, UNDETERMINED, TRAVELING, BRAKE, TESTING,

    MANUAL_CONTROL, CONE, TURRET_STARTUP_MISALIGNMENT
  }


  public enum AutoRoutes {
    NOTHING,
    RED_PICKUP_RIGHT, RED_BALANCE_RIGHT, RED_BALANCE_CENTER, RED_PICKUP_LEFT, RED_NEW_AUTO, RED_PICKUP_BALANCE_RIGHT,
    BLUE_PICKUP_BALANCE_LEFT, BLUE_PICKUP_LEFT, BLUE_BALANCE_CENTER, BLUE_PICKUP_RIGHT
  }
}
