package frc.robot.util.kinematics;

import frc.robot.util.math.Range;

import static frc.robot.Constants.Arm.*;

public class ArmState {
    private final double turretAngle;
    private final double elevatorExtension;
    private final double shoulderAngle;
    private final double wristAngle;
    private final double rotatorAngle;

    private final boolean valid;

    public ArmState(double turretAngle,
                    double elevatorExtension,
                    double shoulderAngle,
                    double wristAngle,
                    double rotatorAngle
    ){
        this.turretAngle = turretAngle;
        this.elevatorExtension = elevatorExtension;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
        this.rotatorAngle = rotatorAngle;

        this.valid = computeValid();
    }

    public ArmState(double elevatorExtension, double shoulderAngle, double wristAngle) {
        this(0, elevatorExtension, shoulderAngle, wristAngle, 0);
    }

    private boolean computeValid() {
        return elevatorRange.isWithin(elevatorExtension) &&
                shoulderRange.isWithin(shoulderAngle) &&
                wristRange.isWithin(wristAngle);
    }

    public boolean isValid() {
        return valid;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getElevatorExtension() {
        return elevatorExtension;
    }

    public double getShoulderAngle() {
        return shoulderAngle;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public double getRotatorAngle() {
        return rotatorAngle;
    }

    @Override
    public String toString() {
        return "ArmState{" +
                "turretAngle=" + turretAngle +
                ", elevatorExtension=" + elevatorExtension +
                ", shoulderAngle=" + shoulderAngle +
                ", wristAngle=" + wristAngle +
                ", rotatorAngle=" + rotatorAngle +
                '}';
    }
}
