package frc.robot.subsystems;

import static frc.robot.Constants.ElementType.*;
import static frc.robot.Constants.Lights.*;
import static frc.robot.Constants.Lights.CONE_RGB;
import static frc.robot.subsystems.Lights.LightState.*;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElementType;
import frc.robot.ShamLib.Candle.CANdleEX;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.commands.WhileDisabledInstantCommand;

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
    addOmniTransition(
        ENABLE_PROX,
        new InstantCommand(
            () -> {
              candle.setLEDs(ENABLE_PROX_RGB);
              prevState = getState();
            }));
    addOmniTransition(
        DISABLE_PROX,
        new InstantCommand(
            () -> {
              candle.setLEDs(DISABLE_PROX_RGB);
              prevState = getState();
            }));
    addOmniTransition(
        GAME_PIECE_GRABBED, new InstantCommand(() -> candle.setLEDs(ELEMENT_GRABBED_RGB)));
    addAnimationTransition(INTAKE_CONE);
    addAnimationTransition(INTAKE_CUBE);
    addAnimationTransition(SOFT_STOP);
    addAnimationTransition(SCORING);

    addOmniTransition(
        LightState.HAVE_CONE_WANT_CUBE, () -> candle.setLEDs(Constants.Lights.HAVE_CONE_WANT_CUBE));
    addOmniTransition(
        LightState.HAVE_CUBE_WANT_CONE, () -> candle.setLEDs(Constants.Lights.HAVE_CUBE_WANT_CONE));

    addAnimationTransition(AUTO);
    //        registerStateCommand(AUTO, new PulseSpeedUpCommand(candle, IDLE_RGB, 15, .125, 1.25)
    //                .andThen(transitionCommand(IDLE)));

    registerStateCommand(ENABLE_PROX, new WaitCommand(1).andThen(transitionCommand(prevState)));
    registerStateCommand(DISABLE_PROX, new WaitCommand(1).andThen(transitionCommand(prevState)));
  }

  public enum LightState {
    DISABLED(DISABLED_ANIMATION), // Where the state machine will start and immediately exit
    AUTO(AUTO_ANIMATION),
    IDLE(null),
    SCORING(SCORING_ANIMATION), // The arm is going to score a game element
    GAME_PIECE_GRABBED(null), // A game piece has been grabbed and is inside the bot
    CONE(null), // The arm will grab a cone next
    HAVE_CONE_WANT_CUBE(null),
    INTAKE_CONE(INTAKE_CONE_ANIMATION),
    CUBE(null), // The arm will grab a cube next
    HAVE_CUBE_WANT_CONE(null),
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
    addOmniTransition(
        state, new WhileDisabledInstantCommand(() -> candle.animate(state.animation)));
  }

  /** Whether the lights can display information about the next game element */
  public boolean canDisplayInfo() {
    return getState() != ENABLE_PROX && getState() != DISABLE_PROX;
  }

  /**
   * Determine the next state of the lights based on the states of the robot
   *
   * @param wanted the element we want to pick up next
   * @param have the element we are currently holding
   * @return the state to send the lights to
   */
  public LightState getStateFromElements(ElementType wanted, ElementType have) {
    if (have == Cone) {
      if (wanted == Cube) {
        return LightState.HAVE_CONE_WANT_CUBE;
      } else return CONE;
    } else if (have == Cube) {
      if (wanted == Cone) {
        return LightState.HAVE_CUBE_WANT_CONE;
      } else return CUBE;
    } else {
      return wanted == Cone ? CONE : CUBE;
    }
  }

  @Override
  protected void onEnable() {
    requestTransition(IDLE);
  }

  @Override
  protected void onAutonomousStart() {
    requestTransition(AUTO);
  }

  @Override
  protected void onDisable() {
    setState(DISABLED);
    candle.animate(DISABLED.animation);
  }

  @Override
  protected void update() {}

  @Override
  protected void determineSelf() {
    setState(DISABLED);
    candle.animate(DISABLED_ANIMATION);
  }
}
