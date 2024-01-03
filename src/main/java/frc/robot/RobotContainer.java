package frc.robot;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static frc.robot.Constants.ElementType.*;
import static frc.robot.Constants.Vision.BASE_LIMELIGHT_POSE;
import static frc.robot.Constants.alliance;
import static frc.robot.subsystems.Drivetrain.SpeedMode.NORMAL;
import static frc.robot.subsystems.Drivetrain.SpeedMode.TURBO;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElementType;
import frc.robot.ShamLib.HID.CommandFlightStick;
import frc.robot.ShamLib.HID.FlightStickImpostor;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;

public class RobotContainer extends StateMachine<RobotContainer.State> {

  // Declare HIDs
  private CommandFlightStick leftStick = new CommandFlightStick(0);
  private CommandFlightStick rightStick = new CommandFlightStick(1);
  private final CommandXboxController operatorCont = new CommandXboxController(2);

  // Declare subsystems
  private final BaseVision baseVision;
  private final ClawVision clawVision;
  private final Drivetrain drivetrain;
  private final Arm arm;
  private final Lights lights;
  private final Turret turret;

  // private final PowerDistribution pd = new PowerDistribution(1,
  // PowerDistribution.ModuleType.kRev);

  private ArmMode currentScoreMode = ArmMode.SEEKING_HIGH;
  private Constants.ElementType nextElement = Cone;
  private boolean trustElementVision = true;

  // Declare autonomous sendable chooser
  private final SendableChooser<Command> autoChooser;

  public RobotContainer(EventLoop checkModulesLoop) {
    super("Robot", State.UNDETERMINED, State.class);

    switch (Constants.currentBuildMode) {
      case REAL:
        arm = new Arm(new ArmIOReal());

        clawVision = new ClawVision(arm::getClawState);

        turret =
            new Turret(
                new TurretIOReal(),
                operatorCont.x(),
                operatorCont.y(),
                clawVision::hasTarget,
                () -> clawVision.getGameElementOffset().getRadians(),
                operatorCont.pov(270),
                operatorCont.pov(90));

        break;
      case SIM:
        leftStick = new CommandFlightStick(1, new FlightStickImpostor(operatorCont, 1, true));
        rightStick = new CommandFlightStick(2, new FlightStickImpostor(operatorCont, 2, false));

        arm = new Arm(new ArmIOSim());

        clawVision = new ClawVision(arm::getClawState);

        turret =
            new Turret(
                new TurretIOSim(),
                operatorCont.x(),
                operatorCont.y(),
                clawVision::hasTarget,
                () -> clawVision.getGameElementOffset().getRadians(),
                operatorCont.pov(270),
                operatorCont.pov(90));

        break;
      default:
        arm = new Arm(new ArmIO() {});

        clawVision = new ClawVision(arm::getClawState);

        turret =
            new Turret(
                new TurretIO() {},
                operatorCont.x(),
                operatorCont.y(),
                clawVision::hasTarget,
                () -> clawVision.getGameElementOffset().getRadians(),
                operatorCont.pov(270),
                operatorCont.pov(90));

        break;
    }

    lights = new Lights();

    baseVision =
        new BaseVision(BASE_LIMELIGHT_POSE, () -> new Rotation2d(turret.getRelativeAngle()));

    drivetrain =
        new Drivetrain(
            () -> -leftStick.getY(),
            () -> -leftStick.getX(),
            () -> {
              return Constants.currentBuildMode != BuildMode.SIM
                  ? -rightStick.getRawAxis(0)
                  : -operatorCont.getRightX();
            },
            baseVision.getPoseSupplier(),
            baseVision.getLLHasTargetSupplier());

    drivetrain.registerMisalignedSwerveTriggers(checkModulesLoop);

    NamedCommands.registerCommand("extend", arm.transitionCommand(ArmMode.SEEKING_HIGH));

    autoChooser = AutoBuilder.buildAutoChooser();

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

    // operatorCont.getHID().setRumble(kBothRumble, 0);
  }

  private void defineTransitions() {
    addOmniTransition(
        State.DISABLED,
        new ParallelCommandGroup(
            arm.transitionCommand(ArmMode.SOFT_STOP),
            turret.transitionCommand(Turret.TurretState.SOFT_STOP),
            lights.transitionCommand(LightState.SOFT_STOP)));

    addOmniTransition(
        State.BRAKE,
        new ParallelCommandGroup(
            drivetrain.transitionCommand(DrivetrainState.X_SHAPE),
            // arm.transitionCommand(ArmMode.SEEKING_STOWED),
            lights.transitionCommand(LightState.IDLE),
            turret.transitionCommand(Turret.TurretState.IDLE)));

    addTransition(State.DISABLED, State.AUTONOMOUS);

    addOmniTransition(
        State.TRAVELING,
        new ParallelCommandGroup(
            drivetrain.transitionCommand(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE),
            turret.transitionCommand(Turret.TurretState.CARDINALS),
            clawVision.transitionCommand(ClawVision.VisionState.ELEMENT_TYPE)));

    addTransition(
        State.TRAVELING,
        State.INTAKING,
        new ParallelCommandGroup(arm.transitionCommand(ArmMode.SEEKING_PICKUP_DOUBLE)));

    addTransition(
        State.TRAVELING,
        State.SCORING,
        new ParallelCommandGroup(
            lights.transitionCommand(LightState.SCORING),
            new InstantCommand(() -> arm.requestTransition(currentScoreMode))));

    // TODO: REMOVE ALL TESTING STUFF
    addOmniTransition(
        State.TESTING,
        new ParallelCommandGroup(
            lights.transitionCommand(LightState.IDLE),
            drivetrain.transitionCommand(DrivetrainState.IDLE),
            turret.transitionCommand(TurretState.IDLE),
            arm.transitionCommand(ArmMode.TESTING)));
  }

  private void defineStateCommands() {
    registerStateCommand(
        State.TRAVELING,
        new RunCommand(
            () -> {
              // Set the element that we have to be what we want next in case the vision fails
              ElementType have =
                  trustElementVision ? clawVision.getCurrentElementType() : nextElement;
              LightState correctState = lights.getStateFromElements(nextElement, have);

              if (lights.getState() != correctState && lights.canDisplayInfo()) {
                lights.requestTransition(correctState);
              }
            }));

    registerStateCommand(
        State.INTAKING,
        new RunCommand(
            () -> {
              // Set the element that we have to be what we want next in case the vision fails
              ElementType have =
                  trustElementVision ? clawVision.getCurrentElementType() : nextElement;
              LightState correctState = lights.getStateFromElements(nextElement, have);

              if (correctState == LightState.CONE) correctState = LightState.INTAKE_CONE;
              else if (correctState == LightState.CUBE) correctState = LightState.INTAKE_CUBE;

              if (lights.getState() != correctState && lights.canDisplayInfo()) {
                lights.requestTransition(correctState);
              }
            }));

    // rightStick.topRight().onTrue(drivetrain.transitionCommand(DrivetrainState.DOCKING));
    // rightStick.topLeft().onTrue(drivetrain.setPositiveDockDirectionCommand(false));

  }

  private void initializeDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Auto Route", autoChooser).withPosition(3, 0).withSize(2, 2);
    driveTab.addString("ALLIANCE", () -> alliance.name()).withPosition(0, 0).withSize(2, 2);
    driveTab.add("SWITCH ALLIANCE", switchAlliance()).withPosition(5, 2).withSize(2, 2);
    driveTab.add("SYNC ALLIANCE", syncAlliance()).withPosition(5, 0).withSize(2, 2);
    // driveTab.addBoolean("Matching Auto", () -> getAlliance() == Constants.alliance)
    // .withPosition(3, 2).withSize(2, 2);

    driveTab.add("+90", zeroTurret(Math.toRadians(90))).withPosition(2, 1).withSize(1, 1);
    driveTab.add("-90", zeroTurret(Math.toRadians(-90))).withPosition(2, 2).withSize(1, 1);
    driveTab
        .addNumber("turret absolute", () -> Math.toDegrees(turret.getTurretAngle()))
        .withPosition(1, 2)
        .withSize(1, 1);
    driveTab
        .addNumber("turret relative", () -> Math.toDegrees(turret.getRelativeAngle()))
        .withPosition(0, 2)
        .withSize(1, 1);

    driveTab.add("TOGGLE ELEMENT VISION", toggleElementVision()).withPosition(7, 0).withSize(2, 2);
    driveTab
        .addBoolean("ELEMENT VISION", () -> trustElementVision)
        .withPosition(7, 2)
        .withSize(2, 2);
  }

  private InstantCommand reZeroTurret() {
    return new WhileDisabledInstantCommand(
        () -> turret.resetAngle(Math.toRadians(alliance.equals(Red) ? -90 : 90)));
  }

  private InstantCommand zeroTurret(double angle) {
    return new WhileDisabledInstantCommand(
        () -> {
          turret.resetAngle(angle);
          Constants.TURRET_ZEROED = true;
        });
  }

  private InstantCommand toggleElementVision() {
    return new WhileDisabledInstantCommand(() -> trustElementVision = !trustElementVision);
  }

  private InstantCommand switchAlliance() {
    return new WhileDisabledInstantCommand(
        () -> {
          alliance = alliance == Red ? Blue : Red;
          Constants.overrideAlliance = true;
        });
  }

  public InstantCommand syncAlliance() {
    return new WhileDisabledInstantCommand(
        () -> {
          Constants.pullAllianceFromFMS(this);
          Constants.overrideAlliance = false;
        });
  }

  private void configureBindings() {

    rightStick.trigger().onTrue(transitionCommand(State.BRAKE));
    rightStick.trigger().onFalse(transitionCommand(State.TRAVELING));

    rightStick
        .topLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (arm.getState() != ArmMode.STOWED)
                    arm.setWristTarget(arm.getWristTarget() - Math.toRadians(2));
                }));
    rightStick
        .topRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (arm.getState() != ArmMode.STOWED)
                    arm.setWristTarget(arm.getWristTarget() + Math.toRadians(2));
                }));

    leftStick.topBase().onTrue(new InstantCommand(drivetrain::resetGyro));

    // Turbo logic
    leftStick
        .trigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  if (arm.getState() == ArmMode.STOWED) drivetrain.setSpeedMode(TURBO);
                }));
    leftStick.trigger().onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(NORMAL)));
    new Trigger(() -> arm.getState() != ArmMode.STOWED)
        .onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(NORMAL)));

    new Trigger(() -> arm.getState() == ArmMode.STOWED)
        .and(leftStick.trigger().negate())
        .onTrue(new InstantCommand(() -> drivetrain.setSpeedMode(TURBO)));

    operatorCont
        .a()
        .onTrue(
            transitionCommand(State.TRAVELING)
                .alongWith(arm.transitionCommand(ArmMode.SEEKING_STOWED)));
    operatorCont
        .b()
        .onTrue(
            new InstantCommand(() -> handleManualRequest(State.INTAKING, TurretState.INTAKING)));

    // Scoring modes for the arm
    operatorCont
        .pov(270)
        .onTrue(
            new InstantCommand(
                () -> {
                  currentScoreMode = ArmMode.LOW_SCORE;
                  handleManualRequest(State.SCORING, TurretState.SCORING);
                }));

    operatorCont
        .pov(0)
        .onTrue(
            new InstantCommand(
                () -> {
                  currentScoreMode = ArmMode.MID_SCORE;
                  handleManualRequest(State.SCORING, TurretState.SCORING);
                }));

    operatorCont
        .pov(90)
        .onTrue(
            new InstantCommand(
                () -> {
                  currentScoreMode = getHighScoreMode();
                  handleManualRequest(State.SCORING, TurretState.SCORING);
                }));

    // Setting the arm to primed position
    operatorCont.pov(180).onTrue(arm.transitionCommand(ArmMode.PRIMED));

    // Claw controls
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

    operatorCont.button(8).onTrue(transitionCommand(State.DISABLED));

    operatorCont
        .pov(90)
        .and(() -> getState() == State.SCORING)
        .onTrue(new InstantCommand(this::handleManualTurretRequest));
    operatorCont
        .pov(270)
        .and(() -> getState() == State.SCORING)
        .onTrue(new InstantCommand(this::handleManualTurretRequest));

    new Trigger(this::lowVoltage)
        .debounce(2)
        .onTrue(new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 1)))
        .onFalse(new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 0)));

    operatorCont
        .leftTrigger(0.8)
        .onTrue(arm.transitionCommand(ArmMode.TELEOP_GROUND_INTERMEDIATE))
        .onFalse(arm.transitionCommand(ArmMode.SEEKING_STOWED));
    operatorCont
        .rightTrigger(0.8)
        .and(() -> operatorCont.getLeftTriggerAxis() > 0.8)
        .onTrue(arm.transitionCommand(ArmMode.NEW_GROUND_PICKUP));

    operatorCont
        .rightTrigger(0.8)
        .onFalse(
            new InstantCommand(
                () -> {
                  if (operatorCont.getLeftTriggerAxis() > 0.8)
                    arm.requestTransition(ArmMode.TELEOP_GROUND_INTERMEDIATE);
                  else arm.transitionCommand(ArmMode.SEEKING_STOWED);
                }));

    // operatorCont.a().onTrue(drivetrain.transitionCommand(DrivetrainState.PATHFINDING));
  }

  public ArmMode getHighScoreMode() {
    if (trustElementVision) {
      if (clawVision.getState() == ClawVision.VisionState.ELEMENT_TYPE
          && clawVision.getCurrentElementType() == Constants.ElementType.Cube) {
        return ArmMode.HIGH_CUBE;
      } else return ArmMode.SEEKING_HIGH;
    } else {
      if (nextElement == Cone) {
        return ArmMode.SEEKING_HIGH;
      } else {
        return ArmMode.HIGH_CUBE;
      }
    }
  }

  private void handleManualTurretRequest() {
    if (isFlag(State.MANUAL_CONTROL)) return;

    if (getState() == State.INTAKING)
      handleManualRequest(State.INTAKING, Turret.TurretState.INTAKING);
    else if (getState() == State.SCORING)
      handleManualRequest(State.SCORING, Turret.TurretState.SCORING);
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
    } else {
      requestTransition(s);
    }
  }

  public Command setTurretToIntake() {
    return new ParallelCommandGroup(
        turret.transitionCommand(TurretState.INTAKING),
        clawVision.transitionCommand(VisionState.CONE_DETECTOR));
  }

  public void scheduleEndgameBuzz() {
    new WaitCommand(103.8).andThen(rumbleLoop(), rumbleLoop(), rumbleLoop()).schedule();
  }

  private Command rumbleLoop() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 1)),
        new WaitCommand(0.25),
        new InstantCommand(() -> operatorCont.getHID().setRumble(kBothRumble, 0)),
        new WaitCommand(0.15));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public boolean lowVoltage() {
    return RobotController.getBatteryVoltage() <= Constants.VOLTAGE_WARNING;
  }

  public Command runTraj(String traj) {
    // return runTraj(paths().get(traj));
    return new InstantCommand();
  }

  public Command runTraj(String traj, boolean resetPose) {
    // return runTraj(paths().get(traj), resetPose);

    return new InstantCommand();
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

  public BaseVision bv() {
    return baseVision;
  }

  public ClawVision cv() {
    return clawVision;
  }

  public Lights lights() {
    return lights;
  }

  public Command waitForReady() {
    return new WaitUntilCommand(
        () -> drivetrain.getState() == DrivetrainState.IDLE && arm.getState() == ArmMode.STOWED);
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
    new WaitCommand(134.7)
        .andThen(
            new ConditionalCommand(
                transitionCommand(State.BRAKE),
                new InstantCommand(),
                () -> drivetrain.getTargetLinearSpeed() < 0.5))
        .schedule();
    new WaitCommand(134.7).andThen(arm.openClaw()).schedule();
  }

  @Override
  protected void onAutonomousStart() {
    registerStateCommand(State.AUTONOMOUS, getAutonomousCommand());
    requestTransition(State.AUTONOMOUS);
  }

  public Pose3d[] getComponentPoses() {
    return new Pose3d[] {
      turret.getOffsetPose(),
      arm.getElevatorPose(turret.getTurretAngle()),
      arm.getShoulderPose(turret.getTurretAngle()),
      arm.getWristPose(turret.getTurretAngle())
    };
  }

  public Pose3d[] getComponentPoseTargets() {
    return new Pose3d[] {
      turret.getOffsetTargetPose(),
      arm.getElevatorPoseTarget(turret.getTurretTarget()),
      arm.getShoulderPoseTarget(turret.getTurretTarget()),
      arm.getWristPoseTarget(turret.getTurretTarget())
    };
  }

  public enum State {
    INTAKING,
    SCORING,
    BALANCING,
    DISABLED,
    AUTONOMOUS,
    UNDETERMINED,
    TRAVELING,
    BRAKE,
    TESTING,

    MANUAL_CONTROL,
    CONE,
    TURRET_STARTUP_MISALIGNMENT
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
