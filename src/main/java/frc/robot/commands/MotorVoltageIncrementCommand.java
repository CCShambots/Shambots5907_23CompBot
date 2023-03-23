package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.motors.pro.EnhancedTalonFXPro;

public class MotorVoltageIncrementCommand extends CommandBase {
    final EnhancedTalonFXPro motor;

    final double incrementSize;
    double increment;

    final double kS = 0;
    final double kG = 0;

    public MotorVoltageIncrementCommand(EnhancedTalonFXPro motor, double incrementSize) {
        this.motor = motor;

        Constants.Testing.RAISE.onTrue(new InstantCommand(() -> increment(1)));
        Constants.Testing.LOWER.onTrue(new InstantCommand(() -> increment(-1)));
        Constants.Testing.STOP.onTrue(new InstantCommand(() -> increment = 0));

        this.incrementSize = incrementSize;

        increment = 0;
    }

    private void increment(int mod) {
        increment = MathUtil.clamp(getUnModifiedIncrement(mod) + ((kS + kG) * Math.signum((getUnModifiedIncrement(mod)))), -12, 12);
    }

    private double getUnModifiedIncrement(int mod) {
        return increment + (incrementSize * mod);
    }

    @Override
    public void initialize() {
        increment = 0;
        motor.setVoltage(0);
    }

    @Override
    public void execute() {
        motor.setVoltage(increment);
        System.out.println("Voltage: " + increment + " Velocity: " + motor.getEncoderVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        increment = 0;
        motor.setVoltage(0);
    }
}
