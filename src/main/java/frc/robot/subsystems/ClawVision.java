package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElementType;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.vision.Limelight;
import frc.robot.subsystems.claw.Claw.ClawState;

import java.util.function.Supplier;

import static frc.robot.Constants.ElementType.*;
import static frc.robot.Constants.Vision.*;
import static frc.robot.subsystems.ClawVision.VisionState.*;

public class ClawVision extends StateMachine<ClawVision.VisionState> {
    private final Limelight ll = new Limelight("limelight-claw");
    private final Supplier<ClawState> clawStateSupplier;

    public ClawVision(Supplier<ClawState> clawStateSupplier) {
        super("Claw Vision", UNDETERMINED, VisionState.class);

        this.clawStateSupplier = clawStateSupplier;

        addOmniTransition(CONE_DETECTOR, () -> setPipeline(CONE_DETECTOR));
        addOmniTransition(CUBE_DETECTOR, () -> setPipeline(CUBE_DETECTOR));
        addOmniTransition(ELEMENT_TYPE, () -> setPipeline(ELEMENT_TYPE));
    }

    public enum VisionState {
        UNDETERMINED(CONE_DETECTOR_PIPELINE),
        CONE_DETECTOR(CONE_DETECTOR_PIPELINE),
        CUBE_DETECTOR(CUBE_DETECTOR_PIPELINE),
        ELEMENT_TYPE(ELEMENT_TYPE_PIPELINE);

        public final int pipelineID;
        VisionState(int pipelineID) {
            this.pipelineID = pipelineID;
        }
    }

    @Override
    protected void update() {
        // GridElement.Type gridType = Constants.gridInterface.getNextElement().getType();

        // if(gridType == GridElement.Type.Both && getState() != CUBE_DETECTOR) requestTransition(CUBE_DETECTOR);
        // if(gridType == GridElement.Type.Cone && getState() != CONE_DETECTOR) requestTransition(CONE_DETECTOR);
        // if(gridType == GridElement.Type.Cube && getState() != CUBE_DETECTOR) requestTransition(CUBE_DETECTOR);
    }

    /**
     * Get the angle offset of a game element detected by the limelight
     * @return the rotation offset
     */
    public Rotation2d getGameElementOffset() {
        return ll.getXOffset();
    }

    public boolean hasTarget() {
        return ll.hasTarget();
    }
    
    public ElementType getCurrentElementType() {
        try {
            if(clawStateSupplier.get() == ClawState.CLOSED) {
                switch (ll.getCurrentElement()) {
                    case 1:
                        return Cone;
                    case 2:
                        return Cube;
                    default:
                        return None;
                }
            } else return None;
        } catch (Exception e) {
            e.printStackTrace();
            return None;
        }
    }

    /**
     * Set the pipeline of the base LL
     * @param state the state to go to
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

    public Command ScoreFirstElementCommand(VisionState coneDetector) {
        return null;
    }

}
