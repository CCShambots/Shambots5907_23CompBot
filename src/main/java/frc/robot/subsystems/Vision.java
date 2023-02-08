package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.vision.Limelight;

import static frc.robot.Constants.Vision.*;
import static frc.robot.subsystems.Vision.VisionState.*;

public class Vision extends StateMachine<Vision.VisionState> {
    private final Limelight baseLL = new Limelight();
    private final Limelight armLL = new Limelight();

    public Vision() {
        super("Vision", Undetermined, VisionState.class);

        addOmniTransition(ElementDetector, () -> setPipeline(ElementDetector));
        addOmniTransition(ConeAngle, () -> setPipeline(ConeAngle));
    }

    public enum VisionState {
        Undetermined(ELEMENT_DETECTOR_PIPELINE),
        ElementDetector(ELEMENT_DETECTOR_PIPELINE),
        ConeAngle(CONE_ORIENTATION_PIPELINE);

        public final int pipelineID;
        VisionState(int pipelineID) {
            this.pipelineID = pipelineID;
        }
    }

    /**
     * Get the pose of the base limelight camera in field space (accounted for pitch already)
     * @return the pose
     */
    public Pose3d getApriltagPose() {
        return baseLL.getPose3d();
    }

    /**
     * Get the angle offset of a game element detected by the limelight
     * @return the rotation offset
     */
    public Rotation2d getGameElementOffset() {
        return armLL.getXOffset(); //TODO: Figure out how this actually works with the vision model
    }

    /**
     * Get the cone angle from the limelight
     * @return the cone angle
     */
    public Rotation2d getConeAngle() {
        return armLL.getConeAngle();
    }

    /**
     * Set the pipeline of the base LL
     * @param state
     */
    private void setPipeline(VisionState state) {
        armLL.setPipeline(state.pipelineID);
    }

    @Override
    protected void onEnable() {
        //No need to do anything here, as the state will become undetermined immediately on disable
    }


    @Override
    protected void onDisable() {
        setState(Undetermined);
        baseLL.setPipeline(APRIL_TAG_PIPELINE); //Make sure the base LL always in apriltag mode
    }

    @Override
    protected void update() {}


    @Override
    protected void determineSelf() {
        baseLL.setPipeline(APRIL_TAG_PIPELINE); //Make sure the base LL always in apriltag mode
        armLL.setPipeline(Undetermined.pipelineID);
        setState(ElementDetector);
    }


    @Override
    protected void additionalSendableData(SendableBuilder builder) {}
}
