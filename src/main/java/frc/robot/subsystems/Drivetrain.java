package frc.robot.subsystems;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.SwerveModule.*;
import static frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleSpeedLevel.*;
import static frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleType.*;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDrivetrain.AutoBalance;
import frc.robot.ShamLib.HID.CommandFlightStick;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.*;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import frc.robot.ShamLib.swerve.module.RealignModuleCommand;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import frc.robot.commands.drivetrain.AutoBalanceCommand;
import frc.robot.commands.drivetrain.DockChargingStationCommand;
import frc.robot.commands.drivetrain.DriveOverChargeStationCommand;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Drivetrain extends StateMachine<Drivetrain.DrivetrainState> {
  private final SwerveDrive drive;
  private final DoubleSupplier x;
  private final DoubleSupplier y;
  private final DoubleSupplier theta;

  private boolean positiveDockDirection =
      true; // Whether the docking should run in a positive or negative direction

  @AutoLogOutput private SwerveModuleState[] moduleStates;

  public Drivetrain(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
    super("Drivetrain", DrivetrainState.UNDETERMINED, DrivetrainState.class);

    this.x = x;
    this.y = y;
    this.theta = theta;

    getOdoPose = this::getPose;

    drive =
        new SwerveDrive(
            Constants.currentBuildMode,
            PIGEON_ID,
            DRIVE_GAINS,
            TURN_GAINS,
            STANDARD_LINEAR_SPEED,
            STANDARD_LINEAR_ACCELERATION,
            STANDARD_ROTATION,
            STANDARD_ROT_ACCEL,
            MAX_TURN_SPEED,
            MAX_TURN_ACCEL,
            new PIDGains(P_HOLDANGLEAUTO, I_HOLDANGLEAUTO, D_HOLDANGLEAUTO),
            new PIDGains(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION),
            !Constants.AT_COMP,
            "drivetrain",
            "",
            Constants.getCurrentLimit(),
            this,
            () -> false,
            0.02,
            ModuleInfo.generateModuleInfo(
                MK4i,
                L2,
                MODULE_1_DRIVE_ID,
                MODULE_1_TURN_ID,
                MODULE_1_ENCODER_ID,
                MODULE_1_OFFSET,
                moduleOffsets[0],
                false,
                false,
                false),
            ModuleInfo.generateModuleInfo(
                MK4i,
                L2,
                MODULE_2_DRIVE_ID,
                MODULE_2_TURN_ID,
                MODULE_2_ENCODER_ID,
                MODULE_2_OFFSET,
                moduleOffsets[1],
                false,
                true,
                true),
            ModuleInfo.generateModuleInfo(
                MK4i,
                L2,
                MODULE_3_DRIVE_ID,
                MODULE_3_TURN_ID,
                MODULE_3_ENCODER_ID,
                MODULE_3_OFFSET,
                moduleOffsets[2],
                false,
                true,
                true),
            ModuleInfo.generateModuleInfo(
                MK4i,
                L2,
                MODULE_4_DRIVE_ID,
                MODULE_4_TURN_ID,
                MODULE_4_ENCODER_ID,
                MODULE_4_OFFSET,
                moduleOffsets[3],
                false,
                true,
                true));

    defineTransitions();
    defineStateCommands();
  }

  private void defineTransitions() {
    addTransition(DrivetrainState.IDLE, DrivetrainState.DRIVING_OVER_CHARGE_STATION);

    addOmniTransition(
        DrivetrainState.X_SHAPE, new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY)));

    addOmniTransition(
        DrivetrainState.IDLE,
        new InstantCommand(
            () -> {
              setAllModules(STOPPED_STATE);
              stopModules();
            }));

    addTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE, DrivetrainState.DOCKING);

    addTransition(
        DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE, DrivetrainState.DRIVING_OVER_CHARGE_STATION);

    addOmniTransition(
        DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE,
        new InstantCommand(() -> setFieldRelative(true)));

    addOmniTransition(DrivetrainState.TRAJECTORY, new InstantCommand());

    addTransition(DrivetrainState.IDLE, DrivetrainState.DOCKING);
    addTransition(DrivetrainState.TRAJECTORY, DrivetrainState.DOCKING);
    addTransition(DrivetrainState.DOCKING, DrivetrainState.BALANCING);

    addTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE, DrivetrainState.DOCKING);
    addTransition(DrivetrainState.DOCKING, DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE);
    addTransition(DrivetrainState.BALANCING, DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE);

    // addTransition(DrivetrainState.GOING_OVER_CHARGE_STATION, DrivetrainState.DOCKING);
    addOmniTransition(DrivetrainState.DOCKING);

    addOmniTransition(DrivetrainState.PATHFINDING);
  }

  private void defineStateCommands() {
    registerStateCommand(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE, getTeleopDriveCommand());

    registerAutoBalanceCommands();

    registerStateCommand(
        DrivetrainState.PATHFINDING,
        drive
            .createPathFindingCommand(
                new Pose2d(
                    Units.feetToMeters(54) / 2, Units.feetToMeters(27) / 2, new Rotation2d()))
            .andThen(transitionCommand(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE)));
  }

  private void registerAutoBalanceCommands() {
    registerStateCommand(
        DrivetrainState.DOCKING,
        new SequentialCommandGroup(
            new InstantCommand(() -> setFieldRelative(true)),
            new DockChargingStationCommand(this, () -> positiveDockDirection ? 1 : -1),
            new ConditionalCommand(
                transitionCommand(DrivetrainState.BALANCING),
                transitionCommand(DrivetrainState.IDLE),
                () -> true)));

    registerStateCommand(
        DrivetrainState.BALANCING,
        new SequentialCommandGroup(
            new AutoBalanceCommand(
                this,
                () -> positiveDockDirection ? 1 : -1,
                AutoBalance.AUTO_BALANCE_GAINS,
                AutoBalance.AUTO_BALANCE_BUFFER_SIZE),
            new ConditionalCommand(
                transitionCommand(DrivetrainState.X_SHAPE),
                transitionCommand(DrivetrainState.IDLE),
                () -> !isFlag(DrivetrainState.DONT_BALANCE))));

    registerStateCommand(
        DrivetrainState.DRIVING_OVER_CHARGE_STATION,
        new SequentialCommandGroup(
            setFlagCommand(DrivetrainState.BEFORE_CHARGE_STATION),
            new DockChargingStationCommand(this, () -> positiveDockDirection ? -1 : 1),
            new DriveOverChargeStationCommand(this, () -> positiveDockDirection ? -1 : 1),
            transitionCommand(DrivetrainState.IDLE)));
  }

  private DriveCommand getTeleopDriveCommand() {
    return new DriveCommand(
        drive,
        x,
        y,
        theta,
        Constants.ControllerConversions.DEADBAND,
        Constants.ControllerConversions.conversionFunction,
        this,
        new SwerveSpeedLimits(
            STANDARD_LINEAR_SPEED,
            STANDARD_LINEAR_ACCELERATION,
            STANDARD_ROTATION,
            STANDARD_ROT_ACCEL),
        new SwerveSpeedLimits(
            MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_ROTATION, MAX_ROT_ACCEL));
  }

  public void enableTeleopAutobalanceControls(CommandFlightStick left, CommandFlightStick right) {
    left.topLeft()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setPositiveDockDirection(false)),
                transitionCommand(DrivetrainState.DOCKING)));

    left.topRight()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setPositiveDockDirection(true)),
                transitionCommand(DrivetrainState.DOCKING)));

    right
        .topLeft()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setPositiveDockDirection(false)),
                transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION)));

    right
        .topRight()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setPositiveDockDirection(true)),
                transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION)));
  }

  public Rotation2d getPitch() {
    return drive.getPitch();
  }

  public Rotation2d getRoll() {
    return drive.getRoll();
  }

  public boolean isFieldRelative() {
    return drive.isFieldRelative();
  }

  public void setFieldRelative(boolean v) {
    drive.setFieldRelative(v);
  }

  public void addVisionMeasurements(TimestampedPoseEstimator.TimestampedVisionUpdate measurement) {
    drive.addTimestampedVisionMeasurements(List.of(measurement));
  }

  public void drive(ChassisSpeeds speeds) {
    drive.drive(speeds);
  }

  public void resetOdometryPose(Pose2d pose) {
    drive.resetOdometryPose(pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return drive.getChassisSpeeds();
  }

  // TODO: remove
  public Field2d getField() {
    return drive.getField();
  }

  /**
   * @param trajectory the trajectory to run
   * @param resetPose whether to reset the pose of the robot at the beginning of the traj
   * @param endState the state to end the trajectory in
   * @return the command to run
   */
  public Command runPath(PathPlannerPath trajectory, boolean resetPose, DrivetrainState endState) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () ->
                registerStateCommand(
                    DrivetrainState.TRAJECTORY,
                    drive
                        .getPathCommand(trajectory)
                        .andThen(
                            new InstantCommand(
                                () -> {
                                  registerStateCommand(
                                      DrivetrainState.TRAJECTORY, new InstantCommand());
                                  requestTransition(endState);
                                })))),
        new InstantCommand(() -> requestTransition(DrivetrainState.TRAJECTORY)));
  }

  public Command runPath(PathPlannerPath trajectory, DrivetrainState endState) {
    return runPath(trajectory, false, endState);
  }

  public Command runPathWithEvents(PathPlannerPath path) {
    return new FollowPathWithEvents(runPath(path, DrivetrainState.IDLE), path, this::getPose);
  }

  public void setModuleStates(SwerveModuleState... states) {
    drive.setModuleStates(states);
  }

  public void setAllModules(SwerveModuleState state) {
    drive.setAllModules(state);
  }

  public Rotation2d getCurrentAngle() {
    return drive.getCurrentAngle();
  }

  public void stopModules() {
    drive.stopModules();
  }

  public void resetGyro(Rotation2d angle) {
    drive.resetGyro(angle);
  }

  public Command resetGyroCommand(Rotation2d angle) {
    return new InstantCommand(() -> drive.resetGyro(angle));
  }

  public void resetGyro() {
    drive.resetGyro();
  }

  @Override
  protected void onTeleopStart() {
    Rotation2d rotation = drive.getPose().getRotation();

    if (Constants.alliance == Alliance.Red) {
      rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
    }

    drive.resetGyro(rotation);

    drive.resetOdometryPose(drive.getPose());

    requestTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE);
  }

  @Override
  protected void onDisable() {
    requestTransition(DrivetrainState.IDLE);
  }

  @Override
  protected void update() {
    drive.update();

    moduleStates = drive.getModuleStates();
  }

  @Override
  protected void determineSelf() {
    setState(DrivetrainState.IDLE);
  }

  public boolean isPositiveDockDirection() {
    return positiveDockDirection;
  }

  public void setPositiveDockDirection(boolean positiveDockDirection) {
    this.positiveDockDirection = positiveDockDirection;
  }

  public Command setPositiveDockDirectionCommand(boolean value) {
    return new InstantCommand(() -> setPositiveDockDirection(value));
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public void registerMisalignedSwerveTriggers(EventLoop loop) {
    for (SwerveModule module : drive.getModules()) {
      loop.bind(
          () -> {
            if (module.isModuleMisaligned() && !isEnabled()) {
              new RealignModuleCommand(module).schedule();
            }
          });
    }
  }

  public void setSpeedMode(SpeedMode mode) {
    drive.setSpeedMode(mode.ordinal());
  }

  public ChassisSpeeds getTargetChassisSpeed() {
    return drive.getTargetChassisSpeeds();
  }

  /**
   * Get the current target linear speed of the chassis
   *
   * @return target speed (in m/s)
   */
  public double getTargetLinearSpeed() {
    ChassisSpeeds target = getTargetChassisSpeed();
    return Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond);
  }

  private double[] poseToArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
  }

  @Override
  public Map<String, Sendable> additionalSendables() {
    return Map.of(
        "field", drive.getField(),
        "module-1", drive.getModules().get(0),
        "module-2", drive.getModules().get(1),
        "module-3", drive.getModules().get(2),
        "module-4", drive.getModules().get(3));
  }

  public enum DrivetrainState {
    UNDETERMINED,
    X_SHAPE,
    FIELD_ORIENTED_TELEOP_DRIVE,
    TRAJECTORY,
    IDLE,
    DOCKING,
    BALANCING,
    DRIVING_OVER_CHARGE_STATION,

    PATHFINDING,

    DONT_BALANCE,
    HIT_ZERO,

    BALANCING_GROUND,
    BEFORE_CHARGE_STATION,
    GOING_OVER_CHARGE_STATION,
    OFF_CHARGE_STATION
  }

  public enum SpeedMode {
    NORMAL,
    TURBO
  }
}
