package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.vision.Limelight;

import static frc.robot.Constants.Vision.*;
import static frc.robot.subsystems.ClawVision.VisionState.*;

public class ClawVision extends StateMachine<ClawVision.VisionState> {
    private final Limelight ll = new Limelight("limelight-arm");

    public ClawVision() {
        super("Vision", Undetermined, VisionState.class);

        addOmniTransition(ConeDetector, () -> setPipeline(ConeDetector));
        addOmniTransition(CubeDetector, () -> setPipeline(CubeDetector));
        addOmniTransition(ConeAngle, () -> setPipeline(ConeAngle));
    }

    public enum VisionState {
        Undetermined(CONE_DETECTOR_PIPELINE),
        ConeDetector(CONE_DETECTOR_PIPELINE),
        CubeDetector(CUBE_DETECTOR_PIPELINE),
        ConeAngle(CONE_ORIENTATION_PIPELINE);

        public final int pipelineID;
        VisionState(int pipelineID) {
            this.pipelineID = pipelineID;
        }
    }

    /**
     * Get the angle offset of a game element detected by the limelight
     * @return the rotation offset
     */
    public Rotation2d getGameElementOffset() {
        return ll.getXOffset(); //TODO: Figure out how this actually works with the vision model
    }

    /**
     * Get the cone angle from the limelight
     * @return the cone angle
     */
    public Rotation2d getConeAngle() {
        return ll.getConeAngle();
    }

    /**
     * Set the pipeline of the base LL
     * @param state
     */
    private void setPipeline(VisionState state) {
        ll.setPipeline(state.pipelineID);
    }


    @Override
    protected void onDisable() {
        setState(Undetermined);
    }


    @Override
    protected void determineSelf() {
        ll.setPipeline(Undetermined.pipelineID);
        setState(ConeDetector);
    }

}
