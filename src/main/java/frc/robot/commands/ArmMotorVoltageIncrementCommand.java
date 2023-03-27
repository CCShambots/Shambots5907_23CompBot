package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.motors.pro.EnhancedTalonFXPro;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.Arm.*;

public class ArmMotorVoltageIncrementCommand extends CommandBase {
    final EnhancedTalonFXPro motor;
    final Arm arm;
    final ArmFeedforward ff;

    final double incrementSize;
    int increment;

    public ArmMotorVoltageIncrementCommand(EnhancedTalonFXPro motor, double incrementSize, Arm arm) {
        this.motor = motor;
        this.incrementSize = incrementSize;
        this.arm = arm;

        ff = new ArmFeedforward(WRIST_GAINS.getS(), WRIST_KG, 0, 0);
        increment = 0;
    }

    private void increment(int mod) {
        increment = MathUtil.clamp(increment + mod, -240, 240);
    }

    private double getVoltage() {
       return (increment * 0.05) + ff.calculate(arm.getShoulderAngle(), Math.signum(increment));
    }

    private double getUnModifiedVoltage() {
        return getVoltage() - ff.calculate(arm.getShoulderAngle(), Math.signum(increment));
    }

    @Override
    public void initialize() {
        Constants.Testing.RAISE.onTrue(new InstantCommand(() -> increment(1)));
        Constants.Testing.LOWER.onTrue(new InstantCommand(() -> increment(-1)));
        Constants.Testing.STOP.onTrue(new InstantCommand(() -> increment = 0));

        increment = 0;
        motor.setVoltage(0);
    }

    @Override
    public void execute() {
        motor.setVoltage(getVoltage());
        System.out.println("Voltage (Unmodified): " + getUnModifiedVoltage() + " Velocity: " + motor.getEncoderVelocity() + " Voltage (Modified): " + getVoltage());
    }

    @Override
    public void end(boolean interrupted) {
        increment = 0;
        motor.setVoltage(0);
    }
}
