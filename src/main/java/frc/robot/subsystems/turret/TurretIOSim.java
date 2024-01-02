package frc.robot.subsystems.turret;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class TurretIOSim extends TurretIOReal {

  public TurretIOSim() {
    super();

    System.out.println("---[init] Turret sim init---");

    PhysicsSim.getInstance().addTalonFX(turretMotor, 0.0001);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.potentiometerAngle = turretMotor.getEncoderPosition();
    inputs.turretMotorAngle = turretMotor.getEncoderPosition();
    inputs.turretTarget = turretMotor.getTarget();
  }

  @Override
  public void changeSpeed(double vel, double accel, double jerk) {}
}
