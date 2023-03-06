package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.MotorOutputConfigs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.pro.MotionMagicTalonFXPro;
import frc.robot.ShamLib.motors.pro.VelocityTalonFXPro;
import frc.robot.ShamLib.motors.rev.PositionSpark;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;
import frc.robot.subsystems.Claw.State;
import frc.robot.util.kinematics.ArmKinematics;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.kinematics.ArmTrajectory;

import java.util.function.BooleanSupplier;

import static com.ctre.phoenixpro.signals.InvertedValue.*;
import static com.ctre.phoenixpro.signals.NeutralModeValue.*;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Arm.*;
import static frc.robot.subsystems.Arm.ArmMode.*;
import static java.lang.Math.*;

public class Arm extends StateMachine<Arm.ArmMode> {

    private final ArmKinematics kinematics = new ArmKinematics(baseToTurret, turretToArm, armToWrist, wristToEndEffector);


    private final MotionMagicTalonFXPro turret = new MotionMagicTalonFXPro(TURRET_ID, TURRET_GAINS, TURRET_INPUT_TO_OUTPUT, TURRET_MAX_VEL, TURRET_MAX_ACCEL);
    private final AnalogPotentiometer turretPotentiometer = new AnalogPotentiometer(TURRET_POT_PORT, TURRET_POT_RATIO, TURRET_ENCODER_OFFSET);

    private final MotionMagicTalonFXPro elevator = new MotionMagicTalonFXPro(ELEVATOR_ID, ELEVATOR_GAINS, ELEVATOR_INPUT_TO_OUTPUT, ELEVATOR_MAX_VEL, ELEVATOR_MAX_ACCEL);

    private final VelocityTalonFXPro shoulder = new VelocityTalonFXPro(SHOULDER_ID, SHOULDER_GAINS, SHOULDER_INPUT_TO_OUTPUT);
    private final ThroughBoreEncoder shoulderEncoder = new ThroughBoreEncoder(SHOULDER_ENCODER_PORT, SHOULDER_ENCODER_OFFSET);
    private final ProfiledPIDController shoulderPID = new ProfiledPIDController(SHOULDER_CONT_GAINS.p, SHOULDER_CONT_GAINS.i, SHOULDER_CONT_GAINS.d, 
        new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL, SHOULDER_MAX_ACCEL)); 
    private double shoulderTarget = toRadians(0);

    private final VelocityTalonFXPro wrist = new VelocityTalonFXPro(WRIST_ID, WRIST_GAINS, WRIST_INPUT_TO_OUTPUT);
    private final ThroughBoreEncoder wristEncoder = new ThroughBoreEncoder(WRIST_ENCODER_PORT, WRIST_ENCODER_OFFSET);
    private final ProfiledPIDController wristPID = new ProfiledPIDController(WRIST_CONT_GAINS.p, WRIST_CONT_GAINS.i, WRIST_CONT_GAINS.d, 
        new TrapezoidProfile.Constraints(WRIST_MAX_VEL, WRIST_MAX_ACCEL));
    private double wristTarget = toRadians(-45);

    private final PositionSpark rotator = new PositionSpark(ROTATOR_ID, kBrushless, ROTATOR_GAINS, ROTATOR_ENCODER_OFFSET, Math.toRadians(1));

    private final ClawVision clawVision = new ClawVision();
    private final Claw claw = new Claw();


    public Arm() {
        super("Arm", UNDETERMINED, ArmMode.class);

        configureHardware();

        addChildSubsystem(clawVision);
        addChildSubsystem(claw);

        defineTransitions();
        registerStateCommands();

        goToArmState(STOWED_POS);

        //TODO: check if abs encoders are zero and disable joint on startup if so
    }

    public Command openClaw() {
        return new InstantCommand(() -> {
            claw.requestTransition(State.OPENED);
        });
    }

    public Command closeClaw() {
        return new InstantCommand(() -> {
            claw.requestTransition(State.CLOSED);
        });
    }

    private void defineTransitions() {
        addOmniTransition(SEEKING_STOWED, new InstantCommand());
        addTransition(SEEKING_STOWED, STOWED);

        addOmniTransition(SOFT_STOP, () -> {
            turret.set(0);
            elevator.set(0);
            shoulder.set(0);
            wrist.set(0);
            rotator.set(0);
        });

        //Easy logics
        addTransition(STOWED, PICKUP_DOUBLE, () -> goToArmState(PICKUP_DOUBLE_POS));
        addTransition(STOWED, LOW_SCORE, () -> goToArmState(LOW_POS));
        addTransition(STOWED, MID_SCORE, () -> goToArmState(MID_POS));

        //hard logics :( 
        addTransition(STOWED, SEEKING_HIGH);
        removeTransition(SEEKING_HIGH, STOWED);
        removeTransition(STOWED, HIGH);
        addTransition(SEEKING_HIGH, HIGH);


    }

    private void registerStateCommands() {
        registerStateCommand(SEEKING_HIGH, 
            new FunctionalCommand(() -> {
                setRotatorTarget(HIGH_POS.getRotatorAngle());
                setWristTarget(HIGH_POS.getWristAngle());
                setShoulderTarget(HIGH_POS.getShoulderAngle());
            }, () -> {}, (interrupted) -> {}, () -> getShoulderAngle() <= SHOULDER_ELEVATOR_THRESHOLD).andThen(
                new InstantCommand(() -> setElevatorTarget(HIGH_POS.getElevatorExtension())),
                new InstantCommand(() -> requestTransition(HIGH))
            )
        );

        registerStateCommand(SEEKING_STOWED,
        new FunctionalCommand(() -> {
            if(getWristAngle() < 0 && getShoulderAngle() < toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT)) setWristTarget(0);
            if(getShoulderAngle() < 0) setShoulderTarget(0);
        }, () -> {}, (interrupted) -> {}, () -> getShoulderAngle() >=0 && (getShoulderAngle() >= toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT) || getWristAngle() >=0 )).andThen(
            new FunctionalCommand(
                () -> setElevatorTarget(STOWED_POS.getElevatorExtension()),
                () -> {},
                (interrupted) -> {},
                () -> getError(getElevatorHeight(), getElevatorTarget()) <= ELEVATOR_TOLERANCE
            ),
            new InstantCommand(() -> goToArmState(STOWED_POS)),
            new InstantCommand(() -> requestTransition(STOWED))
        )
        );


    }

    public enum ArmMode {
        UNDETERMINED, 
        SEEKING_STOWED, STOWED,
        PICKUP_DOUBLE,
        SEEKING_PICKUP_GROUND, PICKUP_GROUND,
        LOW_SCORE, MID_SCORE,
        SEEKING_HIGH, HIGH,
        SOFT_STOP
    }

    private void configureHardware() {
        shoulderEncoder.setInverted(false);
        wristEncoder.setInverted(true);

        MotorOutputConfigs turretConfig = new MotorOutputConfigs();
        turret.getConfigurator().refresh(turretConfig);
        turretConfig.NeutralMode = Brake;
        turretConfig.Inverted = Clockwise_Positive;
        turret.getConfigurator().apply(turretConfig);

        MotorOutputConfigs elevatorConfig = new MotorOutputConfigs();
        elevator.getConfigurator().refresh(elevatorConfig);
        elevatorConfig.NeutralMode = Brake;
        elevatorConfig.Inverted = CounterClockwise_Positive;
        elevator.getConfigurator().apply(elevatorConfig);

        MotorOutputConfigs shoulderConfig = new MotorOutputConfigs();
        shoulder.getConfigurator().refresh(shoulderConfig);
        shoulderConfig.NeutralMode = Brake;
        shoulderConfig.Inverted = CounterClockwise_Positive;
        shoulder.getConfigurator().apply(shoulderConfig);

        MotorOutputConfigs wristConfig = new MotorOutputConfigs();
        wrist.getConfigurator().refresh(wristConfig);
        wristConfig.NeutralMode = Brake;
        wristConfig.Inverted = Clockwise_Positive;
        wrist.getConfigurator().apply(wristConfig);
    }


    public Command calculateTurretFF(Trigger increment, BooleanSupplier interrupt) {
        return turret.calculateKV(TURRET_GAINS.getS(), 0.05, increment, interrupt);
    }
    public Command calculateElevatorFF(Trigger increment, BooleanSupplier interrupt) {
        return elevator.calculateKV(ELEVATOR_GAINS.getS(), 0.05, increment, interrupt);
    }
    public Command calculateShoulderFF(Trigger increment, BooleanSupplier interrupt) {
        return shoulder.calculateKV(SHOULDER_GAINS.getS(), 0.05, increment, interrupt);
    }
    public Command calculateWristFF(Trigger increment, BooleanSupplier interrupt) {
        return wrist.calculateKV(WRIST_GAINS.getS(), 0.25, increment, interrupt);
    }

    public InstantCommand reset() {
        return new InstantCommand(this::pullAbsoluteAngles);
    }

    public Runnable runControlLoops() {
        return () -> {
            //Wrist code
            double wristPIDOutput = wristPID.calculate(wristEncoder.getRadians(), wristTarget);
            
            double wristClampRange = toRadians(90);
            wristPIDOutput = Math.max(-wristClampRange, Math.min(wristClampRange, wristPIDOutput));

            wrist.setTarget(wristPIDOutput + wristPID.getSetpoint().velocity);

            //Shoulder code
            double shoulderPIDOutput = shoulderPID.calculate(shoulderEncoder.getRadians(), shoulderTarget);
            
            double shoulderClampRange = toRadians(12);
            shoulderPIDOutput = Math.max(-shoulderClampRange, Math.min(shoulderClampRange, shoulderPIDOutput));

            shoulder.setTarget(shoulderPIDOutput + shoulderPID.getSetpoint().velocity);
        };
    }
    

    public Command pos1() {
        return new InstantCommand(() -> {
            goToPose(new Pose3d(
                    0.75,
                    0,
                    0.8,
                    new Rotation3d(0, 0, 0)
            ));
        });
    }
    public Command pos2() {
        return new InstantCommand(() -> {
            goToPose(new Pose3d(
                    .56,
                    0,
                    0.96,
                    new Rotation3d(0, -PI/2, 0)
            ));
        });
    }

    public Command followTraj() {
        return new InstantCommand(() -> {
            runTrajectory(new Pose3d(
                    .85, 0, 0.8, new Rotation3d(0, 0, 0)
            ), 0.75).schedule();;
        });

    }

    public Command runTrajectory(Pose3d endPose, double time) {
        return new ArmTrajectory(kinematics, getArmState(), endPose, time).run(this::goToArmState);
    }

    public void goToArmState(ArmState state) {
        if(state.isValid()) {
            setTurretTarget(state.getTurretAngle());
            setElevatorTarget(state.getElevatorExtension());
            setShoulderTarget(state.getShoulderAngle());
            setWristTarget(state.getWristAngle());
            setRotatorTarget(state.getRotatorAngle());
        }
    }

    public void goToPose(Pose3d pose) {
        ArmState target = runIK(pose);
        if(target != null) {
//            pullAbsoluteAngles(); //TODO: Remove when no longer necessary
            goToArmState(target);
        } else {
        }
    }

    public ArmState runIK(Pose3d pose) {
        ArmState[] solutions = kinematics.inverseKinematics(pose);

        return kinematics.chooseIdealState(getArmState(), solutions);
    }

    public Pose3d forwardKinematics(ArmState state) {
        return kinematics.forwardKinematics(state);
    }

    /**
     * Set the target of the turret
     * @param target target angle (in radians)
     */
    public void setTurretTarget(double target) {
        turret.setTarget(target);
    }

    /**
     * Set the target of the elevator
     * @param target target height (in meters)
     */
    public void setElevatorTarget(double target) {
        elevator.setTarget(target);
    }

    /**
     * Set the target of the shoulder
     * @param target target angle (in radians)
     */
    public void setShoulderTarget(double target) {
        shoulderTarget = target;
        shoulderPID.setGoal(target);
    }

    /**
     * Set the target of the wrist
     * @param target target angle (in radians)
     */
    public void setWristTarget(double target) {
        wristTarget = target;
        wristPID.setGoal(target);

    }

    /**
     * Set the target of the rotator
     * @param target target angle (in radians)
     */
    public void setRotatorTarget(double target) {
        rotator.setTarget(target);
    }


    @Override
    protected void determineSelf() {
        pullAbsoluteAngles();

        requestTransition(ArmMode.SEEKING_STOWED);
    }

    @Override
    protected void update() {
        rotator.update();
    }

    @Override
    protected void onEnable() {

    }

    @Override
    protected void onDisable() {

    }

    public Pose3d getArmPose() {
        return kinematics.forwardKinematics(getArmState());
    }

    public ArmState getArmState() {
        return new ArmState(
                getTurretAngle(),
                getElevatorHeight(),
                getShoulderAngle(),
                getWristAngle(),
                getRotatorAngle()
        );
    }

    public double getTurretAngle() {
        return turret.getEncoderPosition();
    }

    public double getTurretTarget() {
        return turret.getTarget();
    }

    public double getElevatorHeight() {
        return elevator.getEncoderPosition();
    }

    public double getElevatorTarget() {
        return elevator.getTarget();
    }

    public double getShoulderAngle() {
        return shoulderEncoder.getRadians();
    }

    public double getShoulderTarget() {
        return shoulder.getTarget();
    }

    public double getWristAngle() {
        return wristEncoder.getRadians();
    }

    public double getWristTarget() {
        return wrist.getTarget();
    }

    public double getRotatorAngle() {
        return rotator.getPosition();
    }
    public double getRotatorTarget() {
        return rotator.getTarget();
    }

    private double getError(double num1, double num2) {
        return abs(num1 - num2);
    }

    public void pullAbsoluteAngles() {
        turret.resetPosition(turretPotentiometer.get() * (PI / 180));
        shoulder.resetPosition(shoulderEncoder.getRadians());
        wrist.resetPosition(wristEncoder.getRadians());
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        // builder.addDoubleProperty("turret/angle", () -> toDegrees(getTurretAngle()), null);
        // builder.addDoubleProperty("turret/target", () -> toDegrees(getTurretTarget()), null);
        // builder.addDoubleProperty("turret/error", () -> getError(toDegrees(getTurretTarget()), toDegrees(getTurretAngle())), null);
        // builder.addDoubleProperty("turret/absolute", () -> turretPotentiometer.get(), null);

        // builder.addDoubleProperty("elevator/output", () -> Units.metersToInches(getElevatorHeight()), null);
        builder.addDoubleProperty("elevator/height", () -> Units.metersToInches(getElevatorHeight()), null);
        // builder.addDoubleProperty("elevator/target", () -> Units.metersToInches(getElevatorTarget()), null);
        // builder.addDoubleProperty("elevator/error", () -> getError(Units.metersToInches(getElevatorTarget()), Units.metersToInches(getElevatorHeight())), null);

        // builder.addDoubleProperty("shoulder/velo", () -> toDegrees(shoulder.getEncoderVelocity()), null);
        // builder.addDoubleProperty("shoulder/angle", () -> toDegrees(shoulder.getEncoderPosition()), null);
        // builder.addDoubleProperty("shoulder/target", () -> toDegrees(getShoulderTarget()), null);
        // builder.addDoubleProperty("shoulder/error", () -> getError(toDegrees(shoulder.getEncoderVelocity()), toDegrees(getShoulderTarget())), null);
        builder.addDoubleProperty("shoulder/absolute", () -> shoulderEncoder.getDegrees(), null);

        // builder.addDoubleProperty("shoulder/shoulder-target-velo", () -> toDegrees(shoulderPID.getSetpoint().velocity), null);
        // builder.addDoubleProperty("shoulder/shoulder-target-pos", () -> toDegrees(shoulderPID.getSetpoint().position), null);

        // builder.addDoubleProperty("wrist/angle", () -> toDegrees(wrist.getEncoderPosition()), null);
        // builder.addDoubleProperty("wrist/velo", () -> toDegrees(wrist.getEncoderVelocity()), null);
        // builder.addDoubleProperty("wrist/target", () -> toDegrees(getWristTarget()), null);
        // builder.addDoubleProperty("wrist/error", () -> getError(toDegrees(wrist.getEncoderVelocity()), toDegrees(getWristTarget())), null);
        builder.addDoubleProperty("wrist/absolute", () -> wristEncoder.getDegrees(), null);

        // builder.addDoubleProperty("wrist/wrist-target-velo", () -> toDegrees(wristPID.getSetpoint().velocity), null);
        // builder.addDoubleProperty("wrist/wrist-target-pos", () -> toDegrees(wristPID.getSetpoint().position), null);

        builder.addDoubleProperty("rotator/output", () -> rotator.getAppliedOutput(), null);
        builder.addDoubleProperty("rotator/angle", () -> toDegrees(getRotatorAngle()), null);
        builder.addDoubleProperty("rotator/target", () -> toDegrees(getRotatorTarget()), null);
        builder.addDoubleProperty("rotator/error", () -> getError(toDegrees(getRotatorTarget()), toDegrees(getRotatorAngle())), null);

        // builder.addDoubleProperty("armpose/x", () -> getArmPose().getX(), null);
        // builder.addDoubleProperty("armpose/y", () -> getArmPose().getY(), null);
        // builder.addDoubleProperty("armpose/z", () -> getArmPose().getZ(), null);
        // builder.addDoubleProperty("armpose/roll", () -> -getArmPose().getRotation().getX(), null);
        // builder.addDoubleProperty("armpose/pitch", () -> -getArmPose().getRotation().getY(), null);
        // builder.addDoubleProperty("armpose/yaw", () -> getArmPose().getRotation().getZ(), null);
    }


}
