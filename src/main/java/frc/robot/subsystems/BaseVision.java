package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShamLib.vision.Limelight;

import java.util.function.Supplier;

public class BaseVision extends SubsystemBase {

    private final Limelight ll = new Limelight("base-ll");

    private Pose3d initialCameraPose;
    private Supplier<Rotation2d> turretAngleSupplier;

    public BaseVision(Pose3d initialCameraPose, Supplier<Rotation2d> turretAngleSupplier) {


    }

    public Pose3d getPose3D(Pose3d limelightPose) {
        Translation3d initialTranslation = initialCameraPose.getTranslation();

        Translation3d newTranslation = initialTranslation.rotateBy(new Rotation3d(0, 0, turretAngleSupplier.get().getRadians()))


        Transform3d transformBy = new Transform3d(
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d(0, 0, turretAngleSupplier.get().getRadians())),
                initialCameraPose).inverse();

        return ll.getPose3d().transformBy(transformBy);
    }


}
