package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShamLib.vision.Limelight;

import java.util.function.Supplier;

public class BaseVision extends SubsystemBase {

    private final Limelight ll = new Limelight("base-ll");
    private final Pose3d initialCameraPose;
    private final Supplier<Rotation2d> turretAngleSupplier;

    /**
     * Create a new instance of the limelight on the base of the robot
     * @param initialCameraPose the pose of the camera with the turret at zero degrees
     * @param turretAngleSupplier supplier for the angle of the turret
     */
    public BaseVision(Pose3d initialCameraPose, Supplier<Rotation2d> turretAngleSupplier) {
        this.initialCameraPose = initialCameraPose;

        this.turretAngleSupplier = turretAngleSupplier;
    }


    /**
     * Get the current pose of the limelight reading
     * @return pose of the limelight
     */
    public Pose3d getPose3D() {
        Translation3d initialTranslation = initialCameraPose.getTranslation();

        //Calculate the translation for where the camera actually is relative to the bot
        Translation3d newTranslation = initialTranslation.rotateBy(new Rotation3d(0, 0, turretAngleSupplier.get().getRadians()));

        //This pose has the correct translation for the camera
        Pose3d poseWithCorrectTranslation = ll.getPose3d().transformBy(new Transform3d(newTranslation, new Rotation3d()));

        //Rotate the camera to face the correct way
        Pose3d endPoint = poseWithCorrectTranslation.transformBy(new Transform3d(new Translation3d(), initialCameraPose.getRotation()));

        return endPoint;
    }


}
