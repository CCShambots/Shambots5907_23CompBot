package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.util.ProxSensor;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.Claw.*;

public class Claw extends StateMachine<Claw.ClawState> {
    final DoubleSolenoid solenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, SOLENOID_ID_1, SOLENOID_ID_2);

    final Compressor compressor = new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

    private ClawState prevState = ClawState.UNDETERMINED;

    private final ProxSensor prox = new ProxSensor(PROX_PORT);
    private final Timer timer = new Timer();
    private boolean proxEnabled = true;

    public Claw() {
        super("Claw", ClawState.UNDETERMINED, ClawState.class);

        compressor.enableDigital();

        defineTransitions();

        registerStateCommands();
    }

    private void defineTransitions() {
        addOmniTransition(ClawState.CLOSED, () -> {
            solenoid.set(flip(SOLENOID_CLAW_OPEN_VALUE));
            prevState = ClawState.CLOSED;
        });
        addOmniTransition(ClawState.OPENED, () -> {
            solenoid.set(SOLENOID_CLAW_OPEN_VALUE);
            prevState = ClawState.OPENED;
            timer.reset();
            timer.start();
        });
    }

    private void registerStateCommands() {
        registerStateCommand(ClawState.OPENED, new RunCommand(() -> {
            if(proxEnabled && prox.isActivated() && timer.get() > 1) requestTransition(ClawState.CLOSED);
        }));
    }

    @Override
    protected void determineSelf() {
        if(prevState == ClawState.UNDETERMINED) {
            DoubleSolenoid.Value solenoidValue = solenoid.get();
            if(solenoidValue == kOff) {
                solenoid.set(flip(SOLENOID_CLAW_OPEN_VALUE));
                setState(ClawState.CLOSED);
            } else if(solenoidValue == SOLENOID_CLAW_OPEN_VALUE) {
                setState(ClawState.OPENED);
            } else {
                setState(ClawState.CLOSED);
            }
        } else {
            setState(prevState);
        }
        
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
            builder.addBooleanProperty("prox", () -> prox.isActivated(), null);
    }

    public void enableProx() {
        proxEnabled = true;
    }

    public void disableProx() {
        proxEnabled = false;
    }

    private DoubleSolenoid.Value flip(DoubleSolenoid.Value value) {
        return value == kForward ? kReverse : kForward;
    }

    public enum ClawState {
        UNDETERMINED, OPENED, CLOSED
    }
}
