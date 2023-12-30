package frc.robot.subsystems.claw;

import static frc.robot.Constants.Claw.COMPRESSOR_ID;
import static frc.robot.Constants.Claw.SOLENOID_ID_1;
import static frc.robot.Constants.Claw.SOLENOID_ID_2;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClawIOReal implements ClawIO{
    private final DoubleSolenoid solenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, SOLENOID_ID_1, SOLENOID_ID_2);

    public ClawIOReal() {

    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
    }

    
}
