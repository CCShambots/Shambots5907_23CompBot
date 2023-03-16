package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.vision.Limelight;
import frc.robot.util.grid.GridElement;

import static frc.robot.Constants.Vision.*;
import static frc.robot.subsystems.ClawVision.VisionState.*;

public class ClawVision extends StateMachine<ClawVision.VisionState> {
    private final Limelight ll = new Limelight("limelight-arm");

    public ClawVision() {
        super("Vision", UNDETERMINED, VisionState.class);

        addOmniTransition(CONE_DETECTOR, () -> setPipeline(CONE_DETECTOR));
        addOmniTransition(CUBE_DETECTOR, () -> setPipeline(CUBE_DETECTOR));
    }

    public enum VisionState {
        UNDETERMINED(CONE_DETECTOR_PIPELINE),
        CONE_DETECTOR(CONE_DETECTOR_PIPELINE),
        CUBE_DETECTOR(CUBE_DETECTOR_PIPELINE);

        public final int pipelineID;
        VisionState(int pipelineID) {
            this.pipelineID = pipelineID;
        }
    }

    @Override
    protected void update() {
        GridElement.Type gridType = Constants.gridInterface.getNextElement().getType();

        if(gridType == GridElement.Type.Both && getState() != CUBE_DETECTOR) requestTransition(CUBE_DETECTOR);
        if(gridType == GridElement.Type.Cone && getState() != CONE_DETECTOR) requestTransition(CONE_DETECTOR);
        if(gridType == GridElement.Type.Cube && getState() != CUBE_DETECTOR) requestTransition(CUBE_DETECTOR);
    }

    /**
     * Get the angle offset of a game element detected by the limelight
     * @return the rotation offset
     */
    public Rotation2d getGameElementOffset() {
        return ll.getXOffset(); //TODO: Figure out how this actually works with the vision model
    }

    public boolean hasTarget() {
        return ll.hasTarget();
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
        setState(UNDETERMINED);
    }


    @Override
    protected void determineSelf() {
        ll.setPipeline(UNDETERMINED.pipelineID);
        setState(CONE_DETECTOR);
    }

}
