package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.ModuleInfo;
import frc.robot.ShamLib.swerve.SwerveDrive;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import frc.robot.ShamLib.swerve.TrajectoryBuilder;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.SwerveModule.*;

public class Drivetrain extends StateMachine<Drivetrain.DrivetrainState> {
    private final SwerveDrive drive;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier theta;

    private final Supplier<Pose3d> llPose;
    private final BooleanSupplier llHasPose;

    public Drivetrain(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, Supplier<Pose3d> llPoseSupplier, BooleanSupplier llHasPose) {
        super("Drivetrain", DrivetrainState.UNDETERMINED, DrivetrainState.class);

        this.x = x;
        this.y = y;
        this.theta = theta;
        this.llPose = llPoseSupplier;
        this.llHasPose = llHasPose;

        drive = new SwerveDrive(
                PIGEON_ID,
                DRIVE_GAINS,
                TURN_GAINS,
                MAX_LINEAR_SPEED,
                MAX_LINEAR_ACCELERATION,
                MAX_TURN_SPEED,
                MAX_TURN_ACCEL,
                new PIDGains(P_HOLDANGLETELE, I_HOLDANGLETELE, D_HOLDANGLETELE),
                new PIDGains(P_HOLDANGLEAUTO, I_HOLDANGLEAUTO, D_HOLDANGLEAUTO),
                new PIDGains(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION),
                true, //TODO: Make this false before comp
                "drivetrain",
                "",
                Constants.CURRENT_LIMIT,
                ModuleInfo.getMK4IL1Module(MODULE_1_DRIVE_ID, MODULE_1_TURN_ID, MODULE_1_ENCODER_ID, MODULE_1_OFFSET, moduleOffsets[0], false),
                ModuleInfo.getMK4IL1Module(MODULE_2_DRIVE_ID, MODULE_2_TURN_ID, MODULE_2_ENCODER_ID, MODULE_2_OFFSET, moduleOffsets[1], false),
                ModuleInfo.getMK4IL1Module(MODULE_3_DRIVE_ID, MODULE_3_TURN_ID, MODULE_3_ENCODER_ID, MODULE_3_OFFSET, moduleOffsets[2], true),
                ModuleInfo.getMK4IL1Module(MODULE_4_DRIVE_ID, MODULE_4_TURN_ID, MODULE_4_ENCODER_ID, MODULE_4_OFFSET, moduleOffsets[3], true)
        );

        defineTransitions();
        defineStateCommands();
    }

    private void defineTransitions() {
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

        addOmniTransition(
                DrivetrainState.FIELD_ORIENTED_TELEOP_DRIVE,
                new InstantCommand(() -> setFieldRelative(true))
        );

        addOmniTransition(
                DrivetrainState.BOT_ORIENTED_TELEOP_DRIVE,
                new InstantCommand(() -> setFieldRelative(false))
        );

        addOmniTransition(DrivetrainState.TRAJECTORY, new InstantCommand());
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
    }

    private DriveCommand getDefaultTeleopDriveCommand() {
        return new DriveCommand(
                drive,
                x,
                y,
                theta,
                MAX_LINEAR_SPEED,
                MAX_LINEAR_ACCELERATION,
                MAX_ROTATION,
                MAX_ROT_ACCEL,
                Constants.ControllerConversions.DEADBAND,
                Constants.ControllerConversions.conversionFunction,
                true,
                this
        );
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
        if(llHasPose.getAsBoolean()) {
            drive.addVisionMeasurement(llPose.get().toPose2d());
        }
    }

    public void drive(ChassisSpeeds speeds, boolean allowHoldAngleChange) {
        drive.drive(speeds, allowHoldAngleChange);
    }

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

    public void resetGyro() {
        drive.resetGyro();
    }

    @Override
    protected void onTeleopStart() {
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

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleArrayProperty("absolute angles", drive::getModuleAbsoluteAngles, null);
        builder.addDoubleProperty("angle", () -> drive.getCurrentAngle().getDegrees(), null);
        builder.addDoubleProperty("hold angle", () -> drive.getHoldAngle().getDegrees(), null);
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        return Map.of(
            "field", drive.getField(),
            "module-1", drive.getModules().get(0),
            "module-2", drive.getModules().get(1),
            "module-3", drive.getModules().get(2),
            "module-4", drive.getModules().get(3)
        );
    }

    public enum DrivetrainState {
        UNDETERMINED, X_SHAPE, FIELD_ORIENTED_TELEOP_DRIVE, BOT_ORIENTED_TELEOP_DRIVE, TRAJECTORY, IDLE
    }
}
