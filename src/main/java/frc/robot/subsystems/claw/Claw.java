package frc.robot.subsystems.claw;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.Claw.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.ShamLib.SMF.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Claw extends StateMachine<Claw.ClawState> {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  final Compressor compressor = new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

  private ClawState prevState = ClawState.UNDETERMINED;

  private final Timer timer = new Timer();
  private boolean proxEnabled = true;

  public Claw(ClawIO io) {
    super("Claw", ClawState.UNDETERMINED, ClawState.class);

    this.io = io;

    compressor.enableDigital();

    defineTransitions();

    registerStateCommands();
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("claw", inputs);
  }

  private void defineTransitions() {
    addOmniTransition(
        ClawState.CLOSED,
        () -> {
          io.close();
          prevState = ClawState.CLOSED;
        });
    addOmniTransition(
        ClawState.OPENED,
        () -> {
          io.open();
          prevState = ClawState.OPENED;
          timer.reset();
          timer.start();
        });
  }

  private void registerStateCommands() {
    registerStateCommand(
        ClawState.OPENED,
        new RunCommand(
            () -> {
              if (proxEnabled && inputs.proxActivated && timer.get() > 1)
                requestTransition(ClawState.CLOSED);
            }));
  }

  @Override
  protected void determineSelf() {
    if (prevState == ClawState.UNDETERMINED) {
      DoubleSolenoid.Value solenoidValue = inputs.solenoidValue;
      if (solenoidValue == kOff) {
        io.close();
        setState(ClawState.CLOSED);
      } else if (solenoidValue == SOLENOID_CLAW_OPEN_VALUE) {
        setState(ClawState.OPENED);
      } else {
        setState(ClawState.CLOSED);
      }
    } else {
      setState(prevState);
    }
  }

  public void enableProx() {
    proxEnabled = true;
  }

  public void disableProx() {
    proxEnabled = false;
  }

  public enum ClawState {
    UNDETERMINED,
    OPENED,
    CLOSED
  }
}
