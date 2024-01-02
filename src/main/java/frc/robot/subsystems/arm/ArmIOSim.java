package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.STOWED_POS;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class ArmIOSim extends ArmIOReal {

  public ArmIOSim() {
    super(true);

    System.out.println("---[init] Arm sim init---");

    PhysicsSim.getInstance().addTalonFX(elevator, 0.0001);
    PhysicsSim.getInstance().addTalonFX(shoulderLeader, 0.0001);
    PhysicsSim.getInstance().addTalonFX(shoulderFollower, 0.0001);
    PhysicsSim.getInstance().addTalonFX(wrist, 0.0001);

    resetWristPos(STOWED_POS.getWristAngle());
    resetShoulderPos(STOWED_POS.getShoulderAngle());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.elevatorTarget = elevator.getTarget();
    inputs.elevatorPos = elevator.getEncoderPosition();

    inputs.shoulderTarget = shoulderLeader.getTarget();
    inputs.shoulderAbsolutePos = shoulderLeader.getEncoderPosition();
    inputs.shoulderMotorPos = shoulderLeader.getEncoderPosition();

    inputs.wristTarget = wrist.getTarget();
    inputs.wristAbsolutePos = wrist.getEncoderPosition();
    inputs.wristMotorPos = wrist.getEncoderPosition();
  }

  @Override
  public void changeShoulderSpeed(double vel, double accel, double jerk) {}

  @Override
  public void changeWristSpeed(double vel, double accel, double jerk) {}
}
