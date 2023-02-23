package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShamLib.vision.Limelight;

import java.util.function.Supplier;

public class BaseVision extends SubsystemBase {

    private final Limelight ll = new Limelight("limelight-base");
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

        //Extract the original translation of the camera
        Translation3d initialTranslation = initialCameraPose.getTranslation();

        //Rotate the camera's translation around the origin to respect the turret angle
        Translation3d newTranslation = initialTranslation.rotateBy(new Rotation3d(0, 0, turretAngleSupplier.get().getRadians()));

        //Combine that translation with the orientation of the camera
        Transform3d transform = new Transform3d(new Pose3d(), new Pose3d(newTranslation, initialCameraPose.getRotation()));

        return ll.getPose3d().transformBy(transform.inverse());
    }


}
