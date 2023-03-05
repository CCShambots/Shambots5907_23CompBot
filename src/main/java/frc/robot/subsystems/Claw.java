package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShamLib.SMF.StateMachine;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static frc.robot.Constants.Claw.*;

public class Claw extends StateMachine<Claw.State> {
    DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_ID_1, SOLENOID_ID_2);

    public Claw() {
        super("Claw", State.UNDETERMINED, State.class);
        // solenoid.set(kForward);
        defineTransitions();
    }


    private void defineTransitions() {
        addOmniTransition(State.CLOSED, new InstantCommand(() -> solenoid.set(SOLENOID_CLAW_OPEN_STATE ? kReverse : kForward)));
        addOmniTransition(State.OPENED, new InstantCommand(() -> solenoid.set(SOLENOID_CLAW_OPEN_STATE ? kForward : kReverse)));
    }

    @Override
    protected void determineSelf() {
        setState(solenoid.get() == ((SOLENOID_CLAW_OPEN_STATE) ? kForward : kReverse) ? State.OPENED : State.CLOSED);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {}

    public enum State {
        UNDETERMINED, OPENED, CLOSED
    }
}
