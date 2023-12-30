package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.motors.talonfx.EnhancedTalonFX;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.Arm.*;

public class ArmMotorVoltageIncrementCommand extends Command {
    final EnhancedTalonFX motor;
    final Arm arm;
    final ArmFeedforward ff;

    final double incrementSize;
    int increment;

    @Deprecated
    public ArmMotorVoltageIncrementCommand(EnhancedTalonFX motor, double incrementSize, Arm arm) {
        this.motor = motor;
        this.incrementSize = incrementSize;
        this.arm = arm;

//        ff = new ArmFeedforward(WRIST_KS, WRIST_KG, 0);
        ff = new ArmFeedforward(0, 0, 0);
        increment = 0;
    }

    private void increment(int mod) {
        increment = MathUtil.clamp(increment + mod, -240, 240);
    }

    private double getVoltage(double angle) {
       return (increment * incrementSize) + ff.calculate(angle, Math.signum(increment));
    }

    private double getUnModifiedVoltage(double angle) {
        return getVoltage(angle) - ff.calculate(angle, Math.signum(increment));
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
        double angle = arm.getShoulderAngle();

        motor.setVoltage(getVoltage(angle));
        System.out.println("Voltage (Unmodified): " + getUnModifiedVoltage(angle) + " Velocity: " + motor.getEncoderVelocity() + " Voltage (Modified): " + getVoltage(angle));
    }

    @Override
    public void end(boolean interrupted) {
        increment = 0;
        motor.setVoltage(0);
    }
}
