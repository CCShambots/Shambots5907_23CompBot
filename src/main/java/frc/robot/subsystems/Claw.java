package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.SMF.StateMachine;

import static frc.robot.Constants.Claw.*;

public class Claw extends StateMachine<Claw.State> {
    Solenoid solenoid = new Solenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, SOLENOID_ID);

    //Compressor compressor = new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

    public Claw() {
        super("Claw", State.UNDETERMINED, State.class);

        //compressor.enableDigital();

        defineTransitions();
    }

    private void defineTransitions() {
        addOmniTransition(State.CLOSED, new InstantCommand(() -> solenoid.set(!SOLENOID_CLAW_OPEN_STATE)));
        addOmniTransition(State.OPENED, new InstantCommand(() -> solenoid.set(SOLENOID_CLAW_OPEN_STATE)));
    }

    @Override
    protected void determineSelf() {
        setState(solenoid.get() == SOLENOID_CLAW_OPEN_STATE ? State.OPENED : State.CLOSED);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {}

    public enum State {
        UNDETERMINED, OPENED, CLOSED
    }
}
