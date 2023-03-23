package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.motors.pro.EnhancedTalonFXPro;

public class MotorVoltageIncrementCommand extends CommandBase {
    final EnhancedTalonFXPro motor;

    final double incrementSize;
    double increment;

    public MotorVoltageIncrementCommand(EnhancedTalonFXPro motor, Trigger raiseTrigger, Trigger lowerTrigger, Trigger stopTrigger, double incrementSize) {
        this.motor = motor;

        raiseTrigger.onTrue(new InstantCommand(() -> increment(1)));
        lowerTrigger.onTrue(new InstantCommand(() -> increment(-1)));
        stopTrigger.onTrue(new InstantCommand(() -> increment = 0));

        this.incrementSize = incrementSize;

        increment = 0;
    }

    private void increment(int mod) {
        increment = Math.min(Math.max(-12, increment + (incrementSize * mod)), 12);
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
