package frc.robot.subsystems.turret;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TurretIOSim extends TurretIOReal {

  private final TalonFXSimState talonSim = turretMotor.getSimState();
  private final DCMotorSim motorSim =
      new DCMotorSim(DCMotor.getFalcon500Foc(1), 1 / TURRET_INPUT_TO_OUTPUT, 0.1793914791);

  public TurretIOSim() {
    super();

    System.out.println("---[init] Turret sim init---");
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    motorSim.setInputVoltage(talonSim.getMotorVoltage());
    motorSim.update(Constants.loopPeriodSecs);

    talonSim.setRawRotorPosition(motorSim.getAngularPositionRotations());
    talonSim.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60.0);

    System.out.println(talonSim.getMotorVoltage());
    System.out.println(turretMotor.getClosedLoopOutput().getValueAsDouble());

    turretMotor.set(1);
    System.out.println(turretMotor.get());

    inputs.potentiometerAngle = turretMotor.getEncoderPosition();
    inputs.turretMotorAngle = turretMotor.getEncoderPosition();
    inputs.turretTarget = turretMotor.getTarget();
  }
}
