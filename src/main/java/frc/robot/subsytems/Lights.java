package frc.robot.subsytems;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.Candle.CANdleEX;
import frc.robot.ShamLib.SMF.StateMachine;

import static frc.robot.Constants.Lights.*;
import static frc.robot.subsytems.Lights.LightState.*;

public class Lights extends StateMachine<Lights.LightState> {

    private final CANdleEX candle = new CANdleEX(CANDLE_ID, brightness, NUM_LEDS);

    public Lights() {
        super("Lights", Disabled, LightState.class);
        candle.animate(Disabled.animation);
    }

    private void addAnimationTransition(LightState state) {
        addOmniTransition(state, new InstantCommand(() -> candle.animate(state.animation)));
    }

    @Override
    protected void onEnable() {

    }

    @Override
    protected void onDisable() {
        setState(Disabled);
        candle.animate(Disabled.animation);
    }

    @Override
    protected void update() {

    }

    @Override
    protected void determineSelf() {
        setState(Idle);
        candle.setLEDs(IDLE_RGB);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

    }

    public enum LightState {
        Disabled(DISABLED_ANIMATION), //Where the state machine will start and immediately exit
        Idle(null),
        ArmDeploying(DEPLOYING_ANIMATION), //The arm is going to score a game element
        ArmScoring(null), //The arm has locked in and is scoring
        GamePieceGrabbed(null), //A game piece has been grabbed and is inside the bot
        UprightCone(null), //The arm will grab an upright cone next
        DownedCone(DOWNED_CONE_ANIMATION), //The arm will grab a downed cone next
        Cube(null); //The arm will grab a cube next

        private final Animation animation;
        LightState(Animation animation) {
            this.animation = animation;
        }
    }
}
