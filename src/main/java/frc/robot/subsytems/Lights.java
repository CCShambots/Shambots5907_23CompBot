package frc.robot.subsytems;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.Candle.CANdleEX;
import frc.robot.ShamLib.SMF.StateMachine;

import static frc.robot.Constants.Lights.*;
import static frc.robot.Constants.Lights.UPRIGHT_CONE_RGB;
import static frc.robot.subsytems.Lights.LightState.*;

public class Lights extends StateMachine<Lights.LightState> {

    private final CANdleEX candle = new CANdleEX(CANDLE_ID, brightness, NUM_LEDS);

    public Lights() {
        super("Lights", DISABLED, LightState.class);
        candle.animate(DISABLED.animation);

        addAnimationTransition(DISABLED);
        addOmniTransition(LightState.UPRIGHT_CONE, new InstantCommand(() -> candle.setLEDs(UPRIGHT_CONE_RGB)));
        addOmniTransition(CUBE, new InstantCommand(() -> candle.setLEDs(CUBE_RGB)));
    }

    public enum LightState {
        DISABLED(DISABLED_ANIMATION), //Where the state machine will start and immediately exit
        ARM_DEPLOYING(DEPLOYING_ANIMATION), //The arm is going to score a game element
        ARM_SCORING(null), //The arm has locked in and is scoring
        GAME_PIECE_GRABBED(null), //A game piece has been grabbed and is inside the bot
        UPRIGHT_CONE(null), //The arm will grab an upright cone next
        DOWNED_CONE(DOWNED_CONE_ANIMATION), //The arm will grab a downed cone next
        CUBE(null); //The arm will grab a cube next

        private final Animation animation;
        LightState(Animation animation) {
            this.animation = animation;
        }
    }

    private void addAnimationTransition(LightState state) {
        addOmniTransition(state, new InstantCommand(() -> candle.animate(state.animation)));
    }

    @Override
    protected void onEnable() {
        requestTransition(UPRIGHT_CONE);
    }

    @Override
    protected void onDisable() {
        setState(DISABLED);
        candle.animate(DISABLED.animation);
    }

    @Override
    protected void update() {

    }

    @Override
    protected void determineSelf() {
        setState(DISABLED);
        candle.animate(DISABLED_ANIMATION);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

    }
}
