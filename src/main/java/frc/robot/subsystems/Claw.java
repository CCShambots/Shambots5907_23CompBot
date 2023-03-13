package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.ShamLib.SMF.StateMachine;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.Claw.*;

public class Claw extends StateMachine<Claw.State> {
    DoubleSolenoid solenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, SOLENOID_ID_1, SOLENOID_ID_2);

    Compressor compressor = new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

    public Claw() {
        super("Claw", State.UNDETERMINED, State.class);

        compressor.enableDigital();

        defineTransitions();
    }

    private void defineTransitions() {
        addOmniTransition(State.CLOSED, () -> solenoid.set(flip(SOLENOID_CLAW_OPEN_VALUE)));
        addOmniTransition(State.OPENED, () -> solenoid.set(SOLENOID_CLAW_OPEN_VALUE));
    }

    @Override
    protected void determineSelf() {
        DoubleSolenoid.Value solenoidValue = solenoid.get();
        if(solenoidValue == kOff) {
            solenoid.set(flip(SOLENOID_CLAW_OPEN_VALUE));
            setState(State.CLOSED);
        } else if(solenoidValue == SOLENOID_CLAW_OPEN_VALUE) {
            setState(State.OPENED);
        } else {
            setState(State.CLOSED);
        }
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
//        builder.addStringProperty("solenoid", () -> solenoid.get().name(), null);
    }

    private DoubleSolenoid.Value flip(DoubleSolenoid.Value value) {
        return value == kForward ? kReverse : kForward;
    }

    public enum State {
        UNDETERMINED, OPENED, CLOSED
    }
}
