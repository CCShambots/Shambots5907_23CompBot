package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElementType;
import frc.robot.ShamLib.AutonomousLoader;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.commands.auto.BaseAutoRoute;
import frc.robot.commands.auto.NothingRoute;
import frc.robot.commands.auto.blue.BlueBalanceCenter;
import frc.robot.commands.auto.blue.BluePickupBalanceLeft;
import frc.robot.commands.auto.blue.BluePickupRight;
import frc.robot.commands.auto.blue.BlueThreeScoreLeft;
import frc.robot.commands.auto.blue.BlueTwoRight;
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
import static frc.robot.Constants.Vision.BASE_LIMELIGHT_POSE;
import static frc.robot.Constants.alliance;
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
  private boolean trustElementVision = true;

  //Declare autonomous loader
  private final AutonomousLoader<BaseAutoRoute, AutoRoutes> autoLoader;

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

    baseVision = new BaseVision(BASE_LIMELIGHT_POSE, () -> new Rotation2d(turret.getRelativeAngle()));

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
        "red-go-score-right",
        "red-go-balance-right",
        "red-return-left",
        "red-balance-center",

        "blue-go-score-left",
        "blue-go-balance-left",
        "blue-return-right"
    );

    loadPaths(2, 4, "red-back-off-right");
    loadPaths(1.25, 1, "red-get-element-right");
    loadPaths(0.75, 0.75, "red-pickup-left");

    //Left two score
    loadPaths(2, 1, "red-get-element-left");
    loadPaths(3, 2, "red-score-element-left");

    //Three score :)
    loadPaths(2, 3, "red-first-score-right");
    loadPaths(4, 2, "red-second-score-right");
    loadPaths(3, 2, "red-get-second-element-right");
    loadPaths(4, 2, "red-third-score-right");
    
    loadPaths(2, 4, "blue-back-off-left");
    loadPaths(1.25, 1, "blue-get-element-left");
    loadPaths(0.75, 0.75, "blue-pickup-right");

    //Three score :)
    loadPaths(2, 3, "blue-first-score-left");
    loadPaths(4, 2, "blue-second-score-left");
    loadPaths(3, 2, "blue-get-second-element-left");
    loadPaths(4, 2, "blue-third-score-left");

    //Right two score
    loadPaths(2, 1, "blue-get-element-right");
    loadPaths(3, 2, "blue-score-element-right");

    autoLoader = instantiateAutoLoader();

    initializeDriveTab();

    addChildSubsystem(drivetrain);
    addChildSubsystem(arm);
    addChildSubsystem(lights);
    addChildSubsystem(clawVision);
    addChildSubsystem(baseVision);
    addChildSubsystem(turret);

    // SmartDashboard.putData("drivetrain", drivetrain);
    SmartDashboard.putData("arm", arm);
    SmartDashboard.putData("turret", turret);
    SmartDashboard.putData("claw", arm.claw());
    SmartDashboard.putData("drivetrain", drivetrain);

    defineTransitions();
    defineStateCommands();

    configureBindings();

    operatorCont.getHID().setRumble(kBothRumble, 0);
  }


  private void defineTransitions() {
    addOmniTransition(State.DISABLED, new ParallelCommandGroup(
            // drivetrain.transitionCommand(DrivetrainState.X_SHAPE),
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
      //Set the element that we have to be what we want next in case the vision fails
      ElementType have = trustElementVision ? clawVision.getCurrentElementType() : nextElement;
      LightState correctState = lights.getStateFromElements(nextElement, have);
      
      if (lights.getState() != correctState && lights.canDisplayInfo()){
        lights.requestTransition(correctState);
      }
    }));

    registerStateCommand(State.INTAKING, new RunCommand(() -> {
      //Set the element that we have to be what we want next in case the vision fails
      ElementType have = trustElementVision ? clawVision.getCurrentElementType() : nextElement;
      LightState correctState = lights.getStateFromElements(nextElement, have);

      if(correctState == LightState.CONE) correctState = LightState.INTAKE_CONE;
      else if(correctState == LightState.CUBE) correctState = LightState.INTAKE_CUBE;

      if (lights.getState() != correctState && lights.canDisplayInfo()) {
        lights.requestTransition(correctState);
      }
    }));

    // rightStick.topRight().onTrue(drivetrain.transitionCommand(DrivetrainState.DOCKING));
    // rightStick.topLeft().onTrue(drivetrain.setPositiveDockDirectionCommand(false));

  }

  private void initializeDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Auto Route", autoLoader.getSendableChooser()).withPosition(3, 0).withSize(2, 2);
    driveTab.addString("ALLIANCE", () -> alliance.name()).withPosition(0, 0).withSize(2, 2);
    driveTab.add("SWITCH ALLIANCE", switchAlliance()).withPosition(5,2).withSize(2, 2);
    driveTab.add("SYNC ALLIANCE", syncAlliance()).withPosition(5,0).withSize(2, 2);
    driveTab.addBoolean("Matching Auto", () -> getAutonomousCommand().getAlliance() == Constants.alliance)
            .withPosition(3, 2).withSize(2, 2);

    driveTab.add("+90", zeroTurret(Math.toRadians(90))).withPosition(2, 1).withSize(1, 1);
    driveTab.add("-90", zeroTurret(Math.toRadians(-90))).withPosition(2, 2).withSize(1, 1);
    driveTab.addNumber("turret absolute", () -> Math.toDegrees(turret.getTurretAngle())).withPosition(1, 2).withSize(1, 1);
    driveTab.addNumber("turret relative", () -> Math.toDegrees(turret.getRelativeAngle())).withPosition(0, 2).withSize(1, 1);
  
    driveTab.add("TOGGLE ELEMENT VISION", toggleElementVision()).withPosition(7, 0).withSize(2, 2);
    driveTab.addBoolean("ELEMENT VISION", () -> trustElementVision).withPosition(7,2).withSize(2, 2);
  }

  private InstantCommand reZeroTurret() {
    return new WhileDisabledInstantCommand(() -> turret.resetAngle(Math.toRadians(alliance.equals(Red) ? -90 : 90)));
  }

  private InstantCommand zeroTurret(double angle) {
    return new WhileDisabledInstantCommand(() -> {
      turret.resetAngle(angle);
      Constants.TURRET_ZEROED = true;
    });
  }

  private InstantCommand toggleElementVision() {
    return new WhileDisabledInstantCommand(() -> trustElementVision = !trustElementVision);
  }

  private AutonomousLoader<BaseAutoRoute, AutoRoutes> instantiateAutoLoader() {
    final AutonomousLoader<BaseAutoRoute, AutoRoutes> autoLoader;

    //Put new auto routes here
    Map<AutoRoutes, BaseAutoRoute> routes = new HashMap<>();

    //Red routes
    routes.putAll(Map.of(
//            RED_TWO_SCORE_RIGHT, new RedTwoScoreRight(this),
            RED_BALANCE_CENTER, new RedBalanceCenter(this),
            RED_PICKUP_LEFT, new RedPickupLeft(this),
            RED_PICKUP_BALANCE_RIGHT, new RedPickupBalanceRight(this),
            RED_THREE_SCORE_RIGHT, new RedThreeScoreRight(this),
            RED_TWO_SCORE_LEFT, new RedTwoLeft(this)
    ));

    //Blue routes
    routes.putAll(Map.of(
//      BLUE_TWO_SCORE_LEFT, new BlueTwoScoreLeft(this),
      BLUE_BALANCE_CENTER, new BlueBalanceCenter(this),
      BLUE_PICKUP_RIGHT, new BluePickupRight(this),
      BLUE_PICKUP_BALANCE_LEFT, new BluePickupBalanceLeft(this),
      BLUE_THREE_SCORE_LEFT, new BlueThreeScoreLeft(this),
      BLUE_TWO_SCORE_RIGHT, new BlueTwoRight(this)
      )
    );

    //Route to do nothing just in case everything has gone wrong
    routes.put(
            NOTHING, new NothingRoute()
    );

    autoLoader = new AutonomousLoader<BaseAutoRoute, AutoRoutes>(routes);

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

  public InstantCommand syncAlliance() {
    return new WhileDisabledInstantCommand(
            () -> {
              Constants.pullAllianceFromFMS(this);
              Constants.overrideAlliance = false;
            }
    );
  }

  private void configureBindings() {

//    dt().enableTeleopAutobalanceControls(leftStick, rightStick);

    // leftStick.topLeft().onTrue(arm.calculateWristFF(leftStick.topRight(), () -> leftStick.topBase().getAsBoolean()));
  //  leftStick.topLeft().onTrue(arm.calculateShoulderFF(leftStick.topRight(), () -> leftStick.topBase().getAsBoolean()));


  //  leftStick.topLeft().onTrue(new InstantCommand(() -> arm.setWristTarget(Math.toRadians(-45))));
  //  leftStick.topBase().onTrue(new InstantCommand(() -> arm.setWristTarget(Math.toRadians(0))));
  //  leftStick.topRight().onTrue(new InstantCommand(() -> arm.setWristTarget(Math.toRadians(45))));

  //  leftStick.topLeft().onTrue(new InstantCommand(() -> arm.setShoulderTarget(Math.toRadians(45))));
  //  leftStick.topBase().onTrue(new InstantCommand(() -> arm.setShoulderTarget(Math.toRadians(90))));
  //  leftStick.topRight().onTrue(new InstantCommand(() -> arm.setShoulderTarget(Math.toRadians(110))));

    rightStick.trigger().onTrue(transitionCommand(State.BRAKE));
    rightStick.trigger().onFalse(transitionCommand(State.TRAVELING));

    rightStick.topLeft().onTrue(new InstantCommand(() -> { if(arm.getState() != ArmMode.STOWED) arm.setWristTarget(arm.getWristTarget()-Math.toRadians(2));}));
    rightStick.topRight().onTrue(new InstantCommand(() -> { if(arm.getState() != ArmMode.STOWED) arm.setWristTarget(arm.getWristTarget()+Math.toRadians(2));}));

    leftStick.topBase().onTrue(new InstantCommand(drivetrain::resetGyro));

    //Turbo logic
    leftStick.trigger()
            .onFalse(new InstantCommand(() -> {if(arm.getState() == ArmMode.STOWED) drivetrain.setSpeedMode(TURBO);}));
    leftStick.trigger().onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(NORMAL)));
    new Trigger(() -> arm.getState() != ArmMode.STOWED)
            .onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(NORMAL)));

    new Trigger(() -> arm.getState() == ArmMode.STOWED).and(() -> !leftStick.trigger().getAsBoolean()).onTrue(
      new InstantCommand(() -> drivetrain.setSpeedMode(TURBO))
    );

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
      currentScoreMode = getHighScoreMode();
      handleManualRequest(State.SCORING, TurretState.SCORING);
    }));

    //Setting the arm to primed position
    operatorCont.pov(180).onTrue(arm.transitionCommand(ArmMode.PRIMED));

    //Claw controls
    operatorCont.leftBumper().onTrue(arm.openClaw());
    operatorCont.rightBumper().onTrue(arm.closeClaw());

    new Trigger(() -> operatorCont.getLeftX() > 0.9)
            .or(() -> operatorCont.getRightX() > 0.9)
            .onTrue(arm.enableClawProx().alongWith(lights.transitionCommand(LightState.ENABLE_PROX)));

    new Trigger(() -> operatorCont.getLeftX() < -0.9)
            .or(() -> operatorCont.getRightX() < -0.9)
            .onTrue(arm.disableClawProx().alongWith(lights.transitionCommand(LightState.DISABLE_PROX)));

    operatorCont.leftStick().onTrue(new InstantCommand(() -> nextElement = Cone));

    operatorCont.rightStick().onTrue(new InstantCommand(() -> nextElement = Cube));

    operatorCont.button(8)
            .onTrue(transitionCommand(State.DISABLED));

    operatorCont.pov(90).and(() -> getState() == State.SCORING).onTrue(new InstantCommand(this::handleManualTurretRequest));
    operatorCont.pov(270).and(() -> getState() == State.SCORING).onTrue(new InstantCommand(this::handleManualTurretRequest));

    new Trigger(this::lowVoltage)
            .debounce(2)
            .onTrue(new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 1)))
            .onFalse(new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 0)));


    operatorCont.leftTrigger(0.8)
      .onTrue(arm.transitionCommand(ArmMode.TELEOP_GROUND_INTERMEDIATE))
      .onFalse(arm.transitionCommand(ArmMode.SEEKING_STOWED));
    operatorCont.rightTrigger(0.8)
      .and(() -> operatorCont.getLeftTriggerAxis() > 0.8)
      .onTrue(arm.transitionCommand(ArmMode.NEW_GROUND_PICKUP));

    operatorCont.rightTrigger(0.8)
    .onFalse(new InstantCommand(() -> {
      if(operatorCont.getLeftTriggerAxis() > 0.8) arm.requestTransition(ArmMode.TELEOP_GROUND_INTERMEDIATE);
      else arm.transitionCommand(ArmMode.SEEKING_STOWED);
    }));
  }

  public ArmMode getHighScoreMode() {
    if(trustElementVision) {
      if(clawVision.getState() == ClawVision.VisionState.ELEMENT_TYPE
              && clawVision.getCurrentElementType() == Constants.ElementType.Cube) {
        return ArmMode.HIGH_CUBE;
      } else return ArmMode.SEEKING_HIGH;
    } else {
      if(nextElement == Cone) {
        return ArmMode.SEEKING_HIGH;
      } else {
        return ArmMode.HIGH_CUBE;
      }
    }
  }

  private void handleManualTurretRequest() {
    if (isFlag(State.MANUAL_CONTROL)) return;

    if (getState() == State.INTAKING) handleManualRequest(State.INTAKING, Turret.TurretState.INTAKING);
    else if (getState() == State.SCORING) handleManualRequest(State.SCORING, Turret.TurretState.SCORING);
  }

  private void handleManualRequest(State s, Turret.TurretState ts) {
    if (!(s == State.SCORING || s == State.INTAKING)) return;

    if (getState() == s) {
      // if (isFlag(State.MANUAL_CONTROL)) {
      //   clearFlag(State.MANUAL_CONTROL);
      //   turret.requestTransition(ts);
      // }
      // else {
      //   setFlag(State.MANUAL_CONTROL);
      //   turret.requestTransition(Turret.TurretState.MANUAL_CONTROL);
      // }
      setFlag(State.MANUAL_CONTROL);
      turret.requestTransition(TurretState.MANUAL_CONTROL);
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

  public BaseAutoRoute getAutonomousCommand() {
    return autoLoader.getCurrentSelection();
  }
//
//  public Runnable runArmControlLoops() {
//    return arm.runControlLoops();
//  }

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

  public Command runTrajWithEvents(String traj, Map<String, Command> eventMap) {
    return drivetrain.runTrajectoryWithEvents(paths().get(traj), eventMap);
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
    new WaitCommand(134.7).andThen(new ConditionalCommand(transitionCommand(State.BRAKE), new InstantCommand(), () -> drivetrain.getTargetLinearSpeed() < 0.5)).schedule();
    new WaitCommand(134.7).andThen(arm.openClaw()).schedule();
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

    RED_TWO_SCORE_RIGHT, 
    RED_BALANCE_RIGHT, 
    RED_PICKUP_BALANCE_CENTER, 
    RED_BALANCE_CENTER,
    RED_PICKUP_LEFT, 
    RED_NEW_AUTO, 
    RED_PICKUP_BALANCE_RIGHT,
    RED_THREE_SCORE_RIGHT,
    RED_TWO_SCORE_LEFT,

    BLUE_PICKUP_BALANCE_LEFT, 
    BLUE_TWO_SCORE_LEFT, 
    BLUE_BALANCE_CENTER, 
    BLUE_PICKUP_RIGHT,
    BLUE_THREE_SCORE_LEFT,
    BLUE_TWO_SCORE_RIGHT
  }
}
