package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.vision.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.Vision.*;

public class BaseVision extends StateMachine<BaseVision.BaseVisionState>{

    private final Limelight ll = new Limelight("limelight-base");
    private final Pose3d initialCameraPose;
    private final Supplier<Rotation2d> turretAngleSupplier;

    /**
     * Create a new instance of the limelight on the base of the robot
     * @param initialCameraPose the pose of the camera with the turret at zero degrees
     * @param turretAngleSupplier supplier for the angle of the turret
     */
    public BaseVision(Pose3d initialCameraPose, Supplier<Rotation2d> turretAngleSupplier) {

        super("Base Vision", BaseVisionState.UNDETERMINED, BaseVisionState.class);

        this.initialCameraPose = initialCameraPose;

        this.turretAngleSupplier = turretAngleSupplier;

        BASE_HAS_TARGET_SUPPLIER = getLLHasTargetSupplier();
        BASE_X_OFFSET_SUPPLIER = getXOffsetSupplier();

        addOmniTransition(BaseVisionState.APRILTAG, new InstantCommand(() -> ll.setPipeline(APRIL_TAG_PIPELINE)));
        addOmniTransition(BaseVisionState.RETROREFLECTIVE, new InstantCommand(() -> ll.setPipeline(RETRO_PIPELINE)));
    }

    /**
     * Get the current pose of the limelight reading
     * @return pose of the limelight (in field space)
     */
    public Pose3d getPose3D() {
        Pose3d initialPose = ll.getPose3d();

        new Pose3d(new Translation3d(initialPose.getX() + Units.feetToMeters(27), initialPose.getY() + Units.feetToMeters(13.5), initialPose.getZ()), initialPose.getRotation());

        return initialPose.transformBy(new Transform3d(new Pose3d(), getLimelightPose()).inverse());
    }

    /**
     * Returns the pose of the liemlight relative to the robot, accounting for the angle of the turret
     * @return relative pose
     */
    public Pose3d getLimelightPose() {
        double turretAngle = turretAngleSupplier.get().plus(Rotation2d.fromDegrees(90)).getRadians(); //Radians

        //Find the actual translation of the camera relative to the robot.
        Translation3d currentTranslation = initialCameraPose.getTranslation().
                rotateBy(new Rotation3d(0, 0, turretAngle));

        Rotation3d initialRotation = initialCameraPose.getRotation();

        return new Pose3d(currentTranslation,
                new Rotation3d(initialRotation.getX(), initialRotation.getY(), initialRotation.getZ() + turretAngle));
    }

    public Supplier<Pose3d> getPoseSupplier() {
        return this::getPose3D;
    }

    public BooleanSupplier getLLHasTargetSupplier() {
        return ll::hasTarget;
    }

    public DoubleSupplier getXOffsetSupplier() {
        return () -> ll.getXOffset().getRadians();
    }

    @Override
    protected void onTeleopStart() {
        transitionCommand(BaseVisionState.APRILTAG);
    }

    @Override
    protected void determineSelf() {
        ll.setPipeline(APRIL_TAG_PIPELINE);
        setState(BaseVisionState.APRILTAG);
    }

    public enum BaseVisionState {
        UNDETERMINED, APRILTAG, RETROREFLECTIVE
    }


}
