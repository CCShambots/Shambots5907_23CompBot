package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShamLib.Candle.CANdleEX;
import frc.robot.ShamLib.SMF.StateMachine;

import static frc.robot.Constants.Lights.*;
import static frc.robot.Constants.Lights.CONE_RGB;
import static frc.robot.subsystems.Lights.LightState.*;

public class Lights extends StateMachine<Lights.LightState> {

    private final CANdleEX candle = new CANdleEX(CANDLE_ID, brightness, NUM_LIGHTS);
    private LightState prevState = CONE;

    public Lights() {
        super("Lights", DISABLED, LightState.class);
        candle.animate(DISABLED.animation);

        addAnimationTransition(DISABLED);
        addOmniTransition(IDLE, new InstantCommand(() -> candle.setLEDs(IDLE_RGB)));
        addOmniTransition(CONE, new InstantCommand(() -> candle.setLEDs(CONE_RGB)));
        addOmniTransition(CUBE, new InstantCommand(() -> candle.setLEDs(CUBE_RGB)));
        addOmniTransition(ENABLE_PROX, new InstantCommand(() -> {
            candle.setLEDs(ENABLE_PROX_RGB);
            prevState = getState();
        }));
        addOmniTransition(DISABLE_PROX, new InstantCommand(() -> {
            candle.setLEDs(DISABLE_PROX_RGB);
            prevState = getState();
        }));
        addOmniTransition(GAME_PIECE_GRABBED, new InstantCommand(() -> candle.setLEDs(ELEMENT_GRABBED_RGB)));
        addAnimationTransition(INTAKE_CONE);
        addAnimationTransition(INTAKE_CUBE);
        addAnimationTransition(SOFT_STOP);
        addAnimationTransition(SCORING);

        registerStateCommand(ENABLE_PROX, new WaitCommand(1).andThen(transitionCommand(prevState)));
        registerStateCommand(DISABLE_PROX, new WaitCommand(1).andThen(transitionCommand(prevState)));
    }

    public enum LightState {
        DISABLED(DISABLED_ANIMATION), //Where the state machine will start and immediately exit
        IDLE(null),
        SCORING(SCORING_ANIMATION), //The arm is going to score a game element
        GAME_PIECE_GRABBED(null), //A game piece has been grabbed and is inside the bot
        CONE(null), //The arm will grab a cone next
        INTAKE_CONE(INTAKE_CONE_ANIMATION),
        CUBE(null), //The arm will grab a cube next
        INTAKE_CUBE(INTAKE_CUBE_ANIMATION),
        ENABLE_PROX(null),
        DISABLE_PROX(null),
        SOFT_STOP(SOFT_STOP_ANIMATION);

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
        requestTransition(IDLE);
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
