package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.ModuleInfo;
import frc.robot.ShamLib.swerve.SwerveDrive;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.SwerveModule.*;

public class Drivetrain extends StateMachine<Drivetrain.DrivetrainState> {
    private final SwerveDrive drive;
    private final CommandXboxController controller;

//    private final Limelight ll = new Limelight();

    private final Transform3d llToBot = new Transform3d(new Pose3d(), Constants.SwerveDrivetrain.limelightPose).inverse();

    public Drivetrain(CommandXboxController controller) {
        super("Drivetrain", DrivetrainState.Undetermined, DrivetrainState.class);

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

        this.controller = controller;

        defineTransitions();
        defineStateCommands();

        SmartDashboard.putData(drive.getField());

        for(int i = 0; i<4; i++) {
            SmartDashboard.putData("/Drivetrain/modules/module-" + (i+1), drive.getModules().get(i));
        }

        // controller.b().onTrue(drive.calculateDriveKS(controller.a()));
        // controller.x().onTrue(drive.calculateDriveKV(DRIVE_GAINS.kS, controller.a(), () -> controller.y().getAsBoolean()));
        controller.a().onTrue(new InstantCommand(() -> setAllModules(new SwerveModuleState(0, new Rotation2d(0)))));
        controller.b().onTrue(new InstantCommand(() -> setAllModules(new SwerveModuleState(1, Rotation2d.fromDegrees(0)))));
        controller.x().onTrue(new InstantCommand(() -> setAllModules(new SwerveModuleState(-1, Rotation2d.fromDegrees(0)))));
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

        addTransition(
                DrivetrainState.Idle,
                DrivetrainState.TeleopDrive,
                new InstantCommand()
        );

        addTransition(
                DrivetrainState.XShape,
                DrivetrainState.TeleopDrive,
                new InstantCommand()
        );
    }

    private void defineStateCommands() {
        registerStateCommand(
                DrivetrainState.TeleopDrive,
                new DriveCommand(
                        drive,
                        () -> -controller.getLeftX(),
                        () -> -controller.getLeftY(),
                        () -> -controller.getRightX(),
                        MAX_LINEAR_SPEED,
                        MAX_LINEAR_ACCELERATION,
                        MAX_ROTATION,
                        MAX_ROT_ACCEL,
                        Constants.ControllerConversions.DEADBAND,
                        Constants.ControllerConversions.conversionFunction,
                        true,
                        this
                )
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

//        if(ll.hasTarget()) {
//            Pose3d rawLLPose = ll.getPose3d();
//
//
//            drive.addVisionMeasurement(rawLLPose.transformBy(llToBot).toPose2d());
//        }
    }

    public void drive(ChassisSpeeds speeds, boolean allowHoldAngleChange) {
        drive.drive(speeds, allowHoldAngleChange);
    }

    public void setModuleStates(SwerveModuleState[] states) {
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

    @Override
    protected void onEnable() {

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

    public enum DrivetrainState {
        Undetermined, XShape, TeleopDrive, Idle
    }
}
