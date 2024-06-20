package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.currentBuildMode;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator.TimestampedVisionUpdate;
import frc.robot.ShamLib.vision.PhotonVision.Apriltag.PVApriltagCam;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class BaseVision extends StateMachine<BaseVision.BaseVisionState> {

  private final PVApriltagCam cam;
  private final Pose3d initialCameraPose;
  private final Supplier<Rotation2d> turretAngleSupplier;

  // Supplier of the current fused estimated pose for trust scaling tags
  Supplier<Pose2d> overallEstimateSupplier = null;

  private final List<Consumer<TimestampedPoseEstimator.TimestampedVisionUpdate>>
      visionUpdateConsumers = new ArrayList<>();

  /**
   * Create a new instance of the limelight on the base of the robot
   *
   * @param initialCameraPose the pose of the camera with the turret at zero degrees
   * @param turretAngleSupplier supplier for the angle of the turret
   */
  public BaseVision(
      Pose3d initialCameraPose, Supplier<Rotation2d> turretAngleSupplier, CamSettings settings) {

    super("Base Vision", BaseVisionState.UNDETERMINED, BaseVisionState.class);

    this.initialCameraPose = initialCameraPose;

    this.turretAngleSupplier = turretAngleSupplier;

    this.cam =
        new PVApriltagCam(
            "limelight-base",
            currentBuildMode,
            getBotToCamTransform(),
            Constants.PhysicalConstants.APRIL_TAG_FIELD_LAYOUT,
            settings.trustCutoff());

    // Multi tag provides much more stable estimates at farther distances
    // cam.setPoseEstimationStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    // Lowest ambiguity is the second most reliable when there aren't multiple tags
    // visible
    // cam.setMultiTagFallbackEstimationStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    applyPreAndPostProcesses(
        cam, settings.ambiguityThreshold(), settings.distanceFromLastEstimateScalar());

    // addOmniTransition(
    // BaseVisionState.APRILTAG, new InstantCommand(() -> ll.setPipeline(APRIL_TAG_PIPELINE)));
  }

  @Override
  protected void update() {
    cam.setBotToCamTransform(getBotToCamTransform());

    cam.update();

    updateConsumers();
  }

  private void updateConsumers() {
    TimestampedPoseEstimator.TimestampedVisionUpdate poseUpdate = getLatestVisionUpdate();

    if (poseUpdate != null) {
      for (var c : visionUpdateConsumers) {
        c.accept(poseUpdate);
      }
    }
  }

  private TimestampedPoseEstimator.TimestampedVisionUpdate getLatestVisionUpdate() {

    Optional<TimestampedVisionUpdate> vision = cam.getLatestEstimate();

    TimestampedVisionUpdate update = vision.orElseGet(() -> null);

    if (update != null) {
      Logger.recordOutput("Vision/" + cam.getName() + "/latestEstimate", update.pose());
      Logger.recordOutput(
          "Vision/" + cam.getName() + "/latestEstimateTimestamp", update.timestamp());
    }

    return update;
  }

  public void setOverallEstimateSupplier(Supplier<Pose2d> supplier) {
    overallEstimateSupplier = supplier;
  }

  public Transform3d getBotToCamTransform() {
    return new Transform3d(new Pose3d(), getLimelightPose());
  }

  /**
   * Add consumers (like the overall robot pose estimator) to receive data about apriltag pose
   * estimations
   *
   * @param consumers consumers to receive pose estimate updates
   */
  public void addVisionUpdateConsumers(
      Consumer<TimestampedPoseEstimator.TimestampedVisionUpdate>... consumers) {
    visionUpdateConsumers.addAll(Arrays.stream(consumers).toList());
  }

  /**
   * Returns the pose of the liemlight relative to the robot, accounting for the angle of the turret
   *
   * @return relative pose
   */
  public Pose3d getLimelightPose() {
    double turretAngle =
        turretAngleSupplier.get().plus(Rotation2d.fromDegrees(90)).getRadians(); // Radians

    // Find the actual translation of the camera relative to the robot.
    Translation3d currentTranslation =
        initialCameraPose.getTranslation().rotateBy(new Rotation3d(0, 0, turretAngle));

    Rotation3d initialRotation = initialCameraPose.getRotation();

    return new Pose3d(
        currentTranslation,
        new Rotation3d(
            initialRotation.getX(), initialRotation.getY(), initialRotation.getZ() + turretAngle));
  }

  private void applyPreAndPostProcesses(
      PVApriltagCam cam, double ambiguityThreshold, double distanceFromLastEstimateScalar) {
    HashMap<Integer, Double[]> ambiguityAverages = new HashMap<>();
    int avgLength = 100;

    cam.setPreProcess(
        (pipelineData) -> {
          if (!pipelineData.hasTargets()) return pipelineData;

          /* VERY IMPORTANT:
           * Clamp received vision timestamps to not pass the RIO time
           * If vision timestamps desync from the RIO, the pose estimate will have huge spasms
           * Issue for almost all of 2024 season - fixed at Champs 2024
           */
          pipelineData.setTimestampSeconds(
              Math.min(Timer.getFPGATimestamp(), pipelineData.getTimestampSeconds()));

          /*
           * Log the ambiguity of each tag the camera can see
           */
          int idx = 0;

          for (var tag : pipelineData.getTargets()) {
            if (!ambiguityAverages.containsKey(tag.getFiducialId())) {
              Double[] arr = new Double[avgLength];
              Arrays.fill(arr, -1.0);
              arr[0] = tag.getPoseAmbiguity();

              ambiguityAverages.put(tag.getFiducialId(), arr);
            } else {
              var arr = ambiguityAverages.get(tag.getFiducialId());
              System.arraycopy(arr, 0, arr, 1, arr.length - 1);
              arr[0] = tag.getPoseAmbiguity();
            }

            double avg = 0;
            double count = 0;
            for (Double a : ambiguityAverages.get(tag.getFiducialId())) {
              if (a >= 0) {
                avg += a;
                count++;
              }
            }

            avg /= count;

            PhotonTrackedTarget target =
                new PhotonTrackedTarget(
                    tag.getYaw(),
                    tag.getPitch(),
                    tag.getArea(),
                    tag.getSkew(),
                    tag.getFiducialId(),
                    tag.getBestCameraToTarget(),
                    tag.getAlternateCameraToTarget(),
                    avg,
                    tag.getMinAreaRectCorners(),
                    tag.getDetectedCorners());

            pipelineData.targets.set(idx, target);

            // Logging the ambiguity for each target can help with debugging potentially problematic
            // tag views
            Logger.recordOutput(
                "Vision/" + cam.getName() + "/target-" + target.getFiducialId() + "-avg-ambiguity",
                target.getPoseAmbiguity());

            idx++;
          }

          // Cut out targets with too high ambiguity
          pipelineData.targets.removeIf(target -> target.getPoseAmbiguity() > ambiguityThreshold);

          return pipelineData;
        });

    cam.setPostProcess(
        (estimate) -> {
          var defaultProcess = cam.defaultPostProcess(estimate);

          // Scale the standard deviations of the pose estimate based on its distance from the
          // current pose estimate
          if (overallEstimateSupplier != null) {
            return new TimestampedPoseEstimator.TimestampedVisionUpdate(
                defaultProcess.timestamp(),
                defaultProcess.pose(),
                defaultProcess
                    .stdDevs()
                    .times(
                        overallEstimateSupplier
                                .get()
                                .getTranslation()
                                .getDistance(defaultProcess.pose().getTranslation())
                            * distanceFromLastEstimateScalar));
          } else {
            return defaultProcess;
          }
        });
  }

  @Override
  protected void onTeleopStart() {
    transitionCommand(BaseVisionState.APRILTAG);
  }

  @Override
  protected void determineSelf() {
    setState(BaseVisionState.APRILTAG);
  }

  public enum BaseVisionState {
    UNDETERMINED,
    APRILTAG
  }

  public record CamSettings(
      String name,
      Pose3d camPose,
      double trustCutoff,
      double ambiguityThreshold,
      double distanceFromLastEstimateScalar,
      double tagDistanceTrustPower,
      double tagDistanceTrustScalar) {}
}
