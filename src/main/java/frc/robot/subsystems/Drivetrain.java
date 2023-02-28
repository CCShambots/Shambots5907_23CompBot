package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.ModuleInfo;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.vision.Limelight;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlanner;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.SwerveModule.*;

public class Drivetrain extends StateMachine<Drivetrain.DrivetrainState> {
    private final SwerveDrive drive;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier theta;

   private final Limelight ll = new Limelight("limelight-base");

    private final Transform3d llToBot = new Transform3d(new Pose3d(), Constants.SwerveDrivetrain.limelightPose).inverse();
    private final Transform2d botToLL = new Transform2d(new Pose2d(), Constants.SwerveDrivetrain.limelightPose.toPose2d());

    public Drivetrain(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        super("Drivetrain", DrivetrainState.Undetermined, DrivetrainState.class);

        this.x = x;
        this.y = y;
        this.theta = theta;

        drive = new SwerveDrive(
                PIGEON_ID,
                DRIVE_GAINS,
                TURN_GAINS,
                MAX_LINEAR_SPEED,
                MAX_TURN_SPEED,
                MAX_TURN_ACCEL,
                new PIDGains(P_HOLDANGLETELE, I_HOLDANGLETELE, D_HOLDANGLETELE),
                new PIDGains(P_HOLDANGLEAUTO, I_HOLDANGLEAUTO, D_HOLDANGLEAUTO),
                new PIDGains(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION),
                true,
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
                DrivetrainState.XShape,
                new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY))
        );

        addOmniTransition(
                DrivetrainState.Idle,
                new InstantCommand(() -> {
                    setAllModules(STOPPED_STATE);
                    stopModules();
                })
        );

        addOmniTransition(
                DrivetrainState.FieldOrientedTeleopDrive,
                new InstantCommand(() -> setFieldRelative(true))
        );

        addOmniTransition(
                DrivetrainState.BotOrientedTeleopDrive,
                new InstantCommand(() -> setFieldRelative(false))
        );

        addOmniTransition(DrivetrainState.Trajectory, new InstantCommand());
    }

    private void defineStateCommands() {
        registerStateCommand(
                DrivetrainState.FieldOrientedTeleopDrive,
                getDefaultTeleopDriveCommand()
        );

        registerStateCommand(
                DrivetrainState.BotOrientedTeleopDrive,
                getDefaultTeleopDriveCommand()
        );

        registerStateCommand(DrivetrainState.Trajectory, drive.getTrajectoryCommand(
            PathPlanner.loadPath("test", Constants.SwerveDrivetrain.MAX_LINEAR_SPEED_AUTO,
              Constants.SwerveDrivetrain.MAX_LINEAR_ACCELERATION_AUTO), true, this)
              .andThen(new InstantCommand(() -> requestTransition(DrivetrainState.FieldOrientedTeleopDrive))));
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

        drive.getField().getObject("ll").setPose(drive.getPose().transformBy(botToLL)); //TODO: Remove

        drive.updateField2dObject();


       if(ll.hasTarget()) {
           Pose3d rawLLPose = ll.getPose3d();
           drive.addVisionMeasurement(rawLLPose.transformBy(llToBot).toPose2d());
       }
    }

    public void drive(ChassisSpeeds speeds, boolean allowHoldAngleChange) {
        drive.drive(speeds, allowHoldAngleChange);
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
        requestTransition(DrivetrainState.FieldOrientedTeleopDrive);
    }

    @Override
    protected void onDisable() {
        requestTransition(DrivetrainState.Idle);
    }

    @Override
    protected void update() {
        updateOdometry();
    }

    @Override
    protected void determineSelf() {
        setState(DrivetrainState.Idle);
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
        Undetermined, XShape, FieldOrientedTeleopDrive, BotOrientedTeleopDrive, Trajectory, Idle
    }
}
