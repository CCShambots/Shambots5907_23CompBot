package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.ShamLib.SMF.StateMachine;

import static frc.robot.Constants.Claw.*;

public class Claw extends StateMachine<Claw.State> {
    Solenoid solenoid = new Solenoid(PCM_ID, PneumaticsModuleType.CTREPCM, SOLENOID_ID);

    public Claw() {
        super("Claw", State.UNDETERMINED, State.class);
    }

    @Override
    protected void onEnable() {
        requestTransition(State.OPENED);
    }

    @Override
    protected void onDisable() {

    }

    @Override
    protected void update() {

    }

    @Override
    protected void determineSelf() {
        //TODO: assuming true is closed and false is opened, switch if testing reveals otherwise
        setState(solenoid.get() ? State.CLOSED : State.OPENED);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

    }

    enum State {
        UNDETERMINED, OPENED, CLOSED
    }
}
