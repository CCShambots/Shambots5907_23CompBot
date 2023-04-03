package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.*;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.commands.drivetrain.AutoBalanceCommand;
import frc.robot.commands.drivetrain.DockChargingStationCommand;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.SwerveModule.*;

public class Drivetrain extends StateMachine<Drivetrain.DrivetrainState> {
    private final SwerveDrive drive;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier theta;

    private final Supplier<Pose3d> llPose;
    private final BooleanSupplier llHasPose;

    private boolean positiveDockDirection = true; //Whether the docking should run in a positive or negative direction

    public Drivetrain(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, Supplier<Pose3d> llPoseSupplier, BooleanSupplier llHasPose) {
        super("Drivetrain", DrivetrainState.UNDETERMINED, DrivetrainState.class);

        this.x = x;
        this.y = y;
        this.theta = theta;
        this.llPose = llPoseSupplier;
        this.llHasPose = llHasPose;

        getOdoPose = this::getPose;

        drive = new SwerveDrive(
                PIGEON_ID,
                DRIVE_GAINS,
                TURN_GAINS,
                STANDARD_LINEAR_SPEED,
                STANDARD_LINEAR_ACCELERATION,
                MAX_TURN_SPEED,
                MAX_TURN_ACCEL,
                new PIDGains(P_HOLDANGLETELE, I_HOLDANGLETELE, D_HOLDANGLETELE),
                new PIDGains(P_HOLDANGLEAUTO, I_HOLDANGLEAUTO, D_HOLDANGLEAUTO),
                new PIDGains(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION),
                false, //TODO: Disable before comp
                "drivetrain",
                "",
                Constants.getCurrentLimit(),
                ModuleInfo.getMK4IL2Module(MODULE_1_DRIVE_ID, MODULE_1_TURN_ID, MODULE_1_ENCODER_ID, MODULE_1_OFFSET, moduleOffsets[0], false),
                ModuleInfo.getMK4IL2Module(MODULE_2_DRIVE_ID, MODULE_2_TURN_ID, MODULE_2_ENCODER_ID, MODULE_2_OFFSET, moduleOffsets[1], false),
                ModuleInfo.getMK4IL2Module(MODULE_3_DRIVE_ID, MODULE_3_TURN_ID, MODULE_3_ENCODER_ID, MODULE_3_OFFSET, moduleOffsets[2], false),
                ModuleInfo.getMK4IL2Module(MODULE_4_DRIVE_ID, MODULE_4_TURN_ID, MODULE_4_ENCODER_ID, MODULE_4_OFFSET, moduleOffsets[3], false)
        );

        defineTransitions();
        defineStateCommands();
    }

    private void defineTransitions() {
        addTransition(DrivetrainState.IDLE, DrivetrainState.DRIVING_OVER_CHARGE_STATION);

        addOmniTransition(
                DrivetrainState.X_SHAPE,
                new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY))
        );

        addOmniTransition(
                DrivetrainState.IDLE,
                new InstantCommand(() -> {
                    setAllModules(STOPPED_STATE);
                    stopModules();
                })
        );

        addTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE, DrivetrainState.DOCKING);

        addTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE, DrivetrainState.DRIVING_OVER_CHARGE_STATION);

        addOmniTransition(
                DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE,
                new InstantCommand(() -> setFieldRelative(true))
        );

        addOmniTransition(
                DrivetrainState.BOT_ORIENTED_TELEOP_DRIVE,
                new InstantCommand(() -> setFieldRelative(false))
        );

        addOmniTransition(DrivetrainState.TRAJECTORY, new InstantCommand());

        addTransition(DrivetrainState.IDLE, DrivetrainState.DOCKING);
        addTransition(DrivetrainState.TRAJECTORY, DrivetrainState.DOCKING);
        addTransition(DrivetrainState.DOCKING, DrivetrainState.BALANCING);

        addTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE, DrivetrainState.DOCKING);
        addTransition(DrivetrainState.DOCKING, DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE);
        addTransition(DrivetrainState.BALANCING, DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE);

        // addTransition(DrivetrainState.GOING_OVER_CHARGE_STATION, DrivetrainState.DOCKING);
        addOmniTransition(DrivetrainState.DOCKING);
    }

    private void defineStateCommands() {
        registerStateCommand(
                DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE,
                getDefaultTeleopDriveCommand()
        );

        registerStateCommand(
                DrivetrainState.BOT_ORIENTED_TELEOP_DRIVE,
                getDefaultTeleopDriveCommand()
        );

        registerAutoBalanceCommands();
    }

    private void registerAutoBalanceCommands() {
        registerStateCommand(DrivetrainState.DOCKING, new SequentialCommandGroup(
                        new InstantCommand(() -> setFieldRelative(true)),
                        new DockChargingStationCommand(this, () -> positiveDockDirection ? 1 : -1),
                        new ConditionalCommand(
                                transitionCommand(DrivetrainState.BALANCING),
                                transitionCommand(DrivetrainState.IDLE),
                                () -> true)
                )
        );

        registerStateCommand(DrivetrainState.BALANCING, new SequentialCommandGroup(
                new AutoBalanceCommand(this, () -> positiveDockDirection ? 1 : -1, AutoBalance.AUTO_BALANCE_GAINS, AutoBalance.AUTO_BALANCE_BUFFER_SIZE),
                new ConditionalCommand(
                        transitionCommand(DrivetrainState.X_SHAPE),
                        transitionCommand(DrivetrainState.IDLE),
                        () -> !isFlag(DrivetrainState.DONT_BALANCE)
                )
        ));

        registerStateCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION, new SequentialCommandGroup(
                setFlagCommand(DrivetrainState.BEFORE_CHARGE_STATION),
                new DockChargingStationCommand(this, () -> positiveDockDirection ? -1 : 1),
                new AutoBalanceCommand(this, () -> positiveDockDirection ? -1 : 1, AutoBalance.AUTO_BALANCE_GAINS, 1, 4),
                new InstantCommand(this::clearFlags),
                setFlagCommand(DrivetrainState.GOING_OVER_CHARGE_STATION),
                new DockChargingStationCommand(this, () -> positiveDockDirection ? -1 : 1, 12), //TODO: change angle possibly
                setFlagCommand(DrivetrainState.BALANCING_GROUND),
                new ParallelRaceGroup(
                        new AutoBalanceCommand(this, () -> positiveDockDirection ? -1 : 1, AutoBalance.AUTO_BALANCE_GAINS, AutoBalance.AUTO_BALANCE_BUFFER_SIZE),
                        new WaitCommand(3)
                ),
                new InstantCommand(this::clearFlags),
                setFlagCommand(DrivetrainState.OFF_CHARGE_STATION),
                transitionCommand(DrivetrainState.DOCKING)
        ));
    }

    private DriveCommand getDefaultTeleopDriveCommand() {
        return new DriveCommand(
                drive,
                x,
                y,
                theta,
                Constants.ControllerConversions.DEADBAND,
                Constants.ControllerConversions.conversionFunction,
                true,
                this,
                new SwerveSpeedLimits(
                        STANDARD_LINEAR_SPEED,
                        STANDARD_LINEAR_ACCELERATION,
                        STANDARD_ROTATION,
                        STANDARD_ROT_ACCEL
                ),
                new SwerveSpeedLimits(
                        MAX_LINEAR_SPEED,
                        MAX_LINEAR_ACCELERATION,
                        MAX_ROTATION,
                        MAX_ROT_ACCEL
                )

        );
    }

    public void enableTeleopAutobalanceControls(CommandFlightStick left, CommandFlightStick right) {
         left.topLeft().onTrue(new SequentialCommandGroup(
                 new InstantCommand(() -> setPositiveDockDirection(false)),
                 transitionCommand(DrivetrainState.DOCKING)
         ));

        left.topRight().onTrue(new SequentialCommandGroup(
                 new InstantCommand(() -> setPositiveDockDirection(true)),
                 transitionCommand(DrivetrainState.DOCKING)
         ));

         right.topLeft().onTrue(new SequentialCommandGroup(
                 new InstantCommand(() -> setPositiveDockDirection(false)),
                 transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION)
         ));

        right.topRight().onTrue(new SequentialCommandGroup(
                 new InstantCommand(() -> setPositiveDockDirection(true)),
                 transitionCommand(DrivetrainState.DRIVING_OVER_CHARGE_STATION)
         ));
    }

    public double getPitch() {
        return drive.getPitch();
    }

    public double getRoll() {
        return drive.getRoll();
    }

    public Command calculateModuleTurn(Trigger increment, BooleanSupplier interrupt) {
        return drive.calculateTurnKV(TURN_GAINS.getS(), increment, interrupt);
    }

    public Command calculateModuleDrive(Trigger increment, Trigger invert, BooleanSupplier interrupt) {
        return drive.calculateDriveKV(DRIVE_GAINS.getS(), increment, invert, interrupt);
    }

    public boolean isFieldRelative() {
        return drive.isFieldRelative();
    }

    public void setFieldRelative(boolean v) {
        drive.setFieldRelative(v);
    }

    public void updateOdometry() {
        drive.updateOdometry();

        drive.updateField2dObject();

        //Only integrate vision measurement if the limelight has a target
        if(getState() == DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE && llHasPose.getAsBoolean()) {
            drive.addVisionMeasurement(llPose.get().toPose2d());
        }
    }

    public void drive(ChassisSpeeds speeds, boolean allowHoldAngleChange) {
        drive.drive(speeds, allowHoldAngleChange);
    }

    //TODO: remove
    public Field2d getField() {return drive.getField();}

    /**
     *
     * @param trajectory the trajectory to run
     * @param resetPose whether to reset the pose of the robot at the beginning of the traj
     * @param endState the state to end the trajectory in
     * @return the command to run
     */
    public Command runTrajectory(PathPlannerTrajectory trajectory, boolean resetPose, DrivetrainState endState) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> registerStateCommand(
                                DrivetrainState.TRAJECTORY,
                                drive.getTrajectoryCommand(trajectory, resetPose)
                                        .andThen(new InstantCommand(() -> {
                                            registerStateCommand(DrivetrainState.TRAJECTORY, new InstantCommand());
                                            requestTransition(endState);
                                        }))
                        )
                ),
                new InstantCommand(() -> requestTransition(DrivetrainState.TRAJECTORY))
        );
    }

    public Command runTrajectory(PathPlannerTrajectory trajectory, DrivetrainState endState) {
        return runTrajectory(trajectory, false, endState);
    }

    public TrajectoryBuilder getTrajectoryBuilder() {
        return drive.getTrajectoryBuilder();
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

        if(Constants.alliance == Alliance.Red) {
            rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
        }

        drive.resetGyro(rotation);
        drive.fixHoldAngle();

        requestTransition(DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE);
    }

    @Override
    protected void onDisable() {
        requestTransition(DrivetrainState.IDLE);
    }

    @Override
    protected void update() {
        updateOdometry();
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
        for(SwerveModule module : drive.getModules()) {
            loop.bind(() -> {
                    if(module.isModuleMisaligned() && !isEnabled()) {
                        new RealignModuleCommand(module).schedule();
                    }
                }
            );  
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
     * @return target speed (in m/s)
     */
    public double getTargetLinearSpeed() {
        ChassisSpeeds target = getTargetChassisSpeed();
        return Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleArrayProperty("absolute angles", drive::getModuleAbsoluteAngles, null);
        builder.addDoubleProperty("angle", () -> drive.getCurrentAngle().getDegrees(), null);
        builder.addDoubleProperty("hold angle", () -> drive.getHoldAngle().getDegrees(), null);

        builder.addDoubleProperty("total angles", () -> Math.abs(getPitch()) + Math.abs(getRoll()), null);

        builder.addDoubleProperty("speed mode", () -> drive.getSpeedMode(), null);
        
        builder.addDoubleProperty("linear-speed", () -> Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond), null);

        builder.addDoubleProperty("pitch", () -> getPitch(), null);
        builder.addDoubleProperty("roll", () -> getRoll(), null);


        // builder.addDoubleProperty("dock-threshold-angle", () -> AutoBalance.DOCK_THRESHOLD, (d) -> {
        //     AutoBalance.DOCK_THRESHOLD = d;
        //     registerAutoBalanceCommands();
        // });

        // builder.addDoubleProperty("balance-no-angle-check-time", () -> AutoBalance.NO_ANGLE_CHECK_TIME, (d) -> {
        //     AutoBalance.NO_ANGLE_CHECK_TIME = d;
        //     registerAutoBalanceCommands();
        // });

        // builder.addIntegerProperty("balance-buffer-size", () -> AutoBalance.AUTO_BALANCE_BUFFER_SIZE, (d) -> {
        //     AutoBalance.AUTO_BALANCE_BUFFER_SIZE = (int) d;
        //     registerAutoBalanceCommands();
        // });

        // builder.addDoubleProperty("balance-p", () -> AutoBalance.AUTO_BALANCE_GAINS.p, (d) -> {
        //     AutoBalance.AUTO_BALANCE_GAINS.p = d;
        //     registerAutoBalanceCommands();
        // });

        // builder.addDoubleProperty("balance-i", () -> AutoBalance.AUTO_BALANCE_GAINS.i, (d) -> {
        //     AutoBalance.AUTO_BALANCE_GAINS.i = d;
        //     registerAutoBalanceCommands();
        // });

        // builder.addDoubleProperty("balance-d", () -> AutoBalance.AUTO_BALANCE_GAINS.d, (d) -> {
        //     AutoBalance.AUTO_BALANCE_GAINS.d = d;
        //     registerAutoBalanceCommands();
        // });

        // builder.addDoubleProperty("balance-speed", () -> AutoBalance.AUTO_BALANCE_SPEED, (d) -> {
        //     AutoBalance.AUTO_BALANCE_SPEED = d;
        //     registerAutoBalanceCommands();
        // });

        // builder.addDoubleProperty("dock-speed", () -> AutoBalance.DOCK_SPEED, (d) -> {
        //     AutoBalance.DOCK_SPEED = d;
        //     registerAutoBalanceCommands();
        // });
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        return Map.of(
            "field", drive.getField()
            // "module-1", drive.getModules().get(0)
            // "module-2", drive.getModules().get(1),
            // "module-3", drive.getModules().get(2),
            // "module-4", drive.getModules().get(3)
        );
    }

    public enum DrivetrainState {
        UNDETERMINED, 
        X_SHAPE, 
        FIELD_ORIENTED_TELEOP_DRIVE, 
        BOT_ORIENTED_TELEOP_DRIVE, 
        TRAJECTORY, 
        IDLE, 
        DOCKING, 
        BALANCING,
        DRIVING_OVER_CHARGE_STATION,

        DONT_BALANCE,

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
