package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.pro.MotionMagicTalonFXPro;
import frc.robot.ShamLib.motors.pro.VelocityTalonFXPro;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;
import frc.robot.commands.arm.MotorVoltageIncrementCommand;
import frc.robot.subsystems.Claw.State;
import frc.robot.util.kinematics.ArmKinematics;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.kinematics.ArmTrajectory;

import java.util.function.BooleanSupplier;

import static com.ctre.phoenixpro.signals.InvertedValue.*;
import static com.ctre.phoenixpro.signals.NeutralModeValue.*;
import static frc.robot.Constants.Arm.*;
import static frc.robot.subsystems.Arm.ArmMode.*;
import static java.lang.Math.*;

public class Arm extends StateMachine<Arm.ArmMode> {

    private final ArmKinematics kinematics = new ArmKinematics(baseToTurret, turretToShoulder, shoulderToWrist, wristToEndEffector);

    private final MotionMagicTalonFXPro elevator = new MotionMagicTalonFXPro(ELEVATOR_ID, ELEVATOR_GAINS, ELEVATOR_INPUT_TO_OUTPUT, ELEVATOR_MAX_VEL, ELEVATOR_MAX_ACCEL);

    private final VelocityTalonFXPro shoulder = new VelocityTalonFXPro(SHOULDER_ID, SHOULDER_GAINS, SHOULDER_INPUT_TO_OUTPUT);
    private final ThroughBoreEncoder shoulderEncoder = new ThroughBoreEncoder(SHOULDER_ENCODER_PORT, SHOULDER_ENCODER_OFFSET);
    private final ProfiledPIDController shoulderPID = new ProfiledPIDController(SHOULDER_CONT_GAINS.p, SHOULDER_CONT_GAINS.i, SHOULDER_CONT_GAINS.d, 
        new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL, SHOULDER_MAX_ACCEL));
    private final ArmFeedforward shoulderFF = new ArmFeedforward(SHOULDER_KS, SHOULDER_KG, SHOULDER_KV);
    private double shoulderTarget = toRadians(0);

    private final VelocityTalonFXPro wrist = new VelocityTalonFXPro(WRIST_ID, WRIST_GAINS, WRIST_INPUT_TO_OUTPUT);
    private final ThroughBoreEncoder wristEncoder = new ThroughBoreEncoder(WRIST_ENCODER_PORT, WRIST_ENCODER_OFFSET);
    private final ProfiledPIDController wristPID = new ProfiledPIDController(WRIST_CONT_GAINS.p, WRIST_CONT_GAINS.i, WRIST_CONT_GAINS.d, 
        new TrapezoidProfile.Constraints(WRIST_MAX_VEL, WRIST_MAX_ACCEL));
    private double wristTarget = toRadians(-45);

    // private final PositionSpark rotator = new PositionSpark(ROTATOR_ID, kBrushless, ROTATOR_GAINS, ROTATOR_ENCODER_OFFSET, Math.toRadians(1));

    private final ClawVision clawVision = new ClawVision();
    private final Claw claw = new Claw();

    private ArmState currentArmState = STOWED_POS;


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
        addOmniTransition(SEEKING_STOWED);
        addTransition(SEEKING_STOWED, STOWED);

        addOmniTransition(SOFT_STOP, () -> {
            elevator.set(0);
            shoulder.set(0);
            wrist.set(0);
            // rotator.set(0);
        });

        //Easy logics
        addTransition(STOWED, PICKUP_DOUBLE, new InstantCommand(() -> goToArmState(PICKUP_DOUBLE_POS)).alongWith(claw.transitionCommand(State.OPENED)));
        addTransition(STOWED, LOW_SCORE, () -> goToArmState(LOW_POS));
        addTransition(STOWED, MID_SCORE, () -> goToArmState(MID_POS));
        addTransition(STOWED, HIGH_CUBE, () -> goToArmState(HIGH_CUBE_POS));

        addTransition(STOWED, SEEKING_POSE);
        addTransition(SEEKING_POSE, AT_POSE);

        //hard logics :( 
        addTransition(STOWED, SEEKING_HIGH);
        removeTransition(SEEKING_HIGH, STOWED);
        addTransition(SEEKING_HIGH, HIGH);
        addTransition(HIGH_CUBE, SEEKING_PICKUP_GROUND);

        addTransition(STOWED, SEEKING_PICKUP_GROUND);
        addTransition(HIGH, SEEKING_PICKUP_GROUND);
        removeTransition(SEEKING_PICKUP_GROUND, STOWED);
        addTransition(SEEKING_PICKUP_GROUND, PICKUP_GROUND);
        addTransition(HIGH_CUBE, SEEKING_PICKUP_GROUND);
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
            if(getShoulderAngle() < 0) setShoulderTarget(15);
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

        registerStateCommand(SEEKING_PICKUP_GROUND,
                    new FunctionalCommand(() -> {
                        setRotatorTarget(GROUND_PICKUP_POS.getRotatorAngle());
                        setWristTarget(GROUND_PICKUP_POS.getWristAngle());
                        setShoulderTarget(GROUND_PICKUP_POS.getShoulderAngle());
                    }, () -> {}, (interrupted) -> {}, () -> getShoulderAngle() <= SHOULDER_ELEVATOR_THRESHOLD).andThen(
                        new InstantCommand(() -> setElevatorTarget(GROUND_PICKUP_POS.getElevatorExtension())),
                        new InstantCommand(() -> requestTransition(PICKUP_GROUND))
                    )
        );

        registerStateCommand(SEEKING_POSE, 
            new FunctionalCommand(() -> {
                setRotatorTarget(currentArmState.getRotatorAngle());
                setWristTarget(currentArmState.getWristAngle());
                setShoulderTarget(currentArmState.getShoulderAngle());
            }, () -> {}, (interrupted) -> {}, () -> getShoulderAngle() <= SHOULDER_ELEVATOR_THRESHOLD).andThen(
                new InstantCommand(() -> setElevatorTarget(currentArmState.getElevatorExtension())),
                new InstantCommand(() -> requestTransition(AT_POSE))
            )
        );

        registerStateCommand(TESTING, new MotorVoltageIncrementCommand(
                shoulder,
                new Trigger(() -> false),
                new Trigger(() -> false),
                new Trigger(() -> true),
                0.05
        ));
    }


    public enum ArmMode {
        UNDETERMINED, 
        SEEKING_STOWED, STOWED,
        PICKUP_DOUBLE,
        SEEKING_PICKUP_GROUND, PICKUP_GROUND,
        LOW_SCORE, MID_SCORE,
        SEEKING_HIGH, HIGH,
        HIGH_CUBE,
        SOFT_STOP,

        SEEKING_POSE,
        AT_POSE,

        TESTING
    }

    private void configureHardware() {
        shoulderEncoder.setInverted(false);
        wristEncoder.setInverted(true);


        elevator.configure(Brake, CounterClockwise_Positive);

        shoulder.configure(Brake, CounterClockwise_Positive);

        wrist.configure(Brake, Clockwise_Positive);
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
        return new InstantCommand(() -> {
            pullAbsoluteAngles();
            wristPID.reset(getWristAngle());
            shoulderPID.reset(getShoulderAngle());
        });
    }

    public Runnable runControlLoops() {
        return () -> {
            //Wrist code
            double wristPIDOutput = wristPID.calculate(wristEncoder.getRadians(), wristTarget);
            
            double wristClampRange = toRadians(90);
            wristPIDOutput = Math.max(-wristClampRange, Math.min(wristClampRange, wristPIDOutput));

            wrist.setTarget(wristPIDOutput + wristPID.getSetpoint().velocity);

            //Shoulder code
            double shoulderPIDOutput = shoulderPID.calculate(getShoulderAngle(), shoulderTarget);
            double shoulderFFOutput = shoulderFF.calculate(shoulderPID.getSetpoint().position,  shoulderPID.getSetpoint().velocity);

            shoulder.setVoltage(shoulderPIDOutput + shoulderFFOutput);
        };
    }

    public Command runTrajectory(Pose3d endPose, double time) {
        return new ArmTrajectory(kinematics, getArmState(), endPose, time).run(this::goToArmState);
    }

    public void goToArmState(ArmState state) {
        if(state.isValid()) {
            setElevatorTarget(state.getElevatorExtension());
            setShoulderTarget(state.getShoulderAngle());
            setWristTarget(state.getWristAngle());
        }
    }

    public void goToPose(Pose3d pose) {
        ArmState target = runIK(pose);

        System.out.println(target);

        if(target != null) {
            System.out.println(target);

            currentArmState = target;
            requestTransition(SEEKING_POSE);
            // goToArmState(target);
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
        // rotator.setTarget(target);
    }


    @Override
    protected void determineSelf() {
        pullAbsoluteAngles();

        requestTransition(ArmMode.SEEKING_STOWED);
    }

    @Override
    protected void update() {
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
                getElevatorHeight(),
                getShoulderAngle(),
                getWristAngle()
        );
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

    private double getError(double num1, double num2) {
        return abs(num1 - num2);
    }

    public void pullAbsoluteAngles() {
        shoulder.resetPosition(shoulderEncoder.getRadians());
        wrist.resetPosition(wristEncoder.getRadians());
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

        // builder.addDoubleProperty("elevator/output", () -> Units.metersToInches(getElevatorHeight()), null);
        builder.addDoubleProperty("elevator/height", () -> Units.metersToInches(getElevatorHeight()), null);
        // builder.addDoubleProperty("elevator/target", () -> Units.metersToInches(getElevatorTarget()), null);
        // builder.addDoubleProperty("elevator/error", () -> getError(Units.metersToInches(getElevatorTarget()), Units.metersToInches(getElevatorHeight())), null);

        // builder.addDoubleProperty("shoulder/velo", () -> toDegrees(shoulder.getEncoderVelocity()), null);
        // builder.addDoubleProperty("shoulder/angle", () -> toDegrees(shoulder.getEncoderPosition()), null);
        // builder.addDoubleProperty("shoulder/target", () -> toDegrees(getShoulderTarget()), null);
        // builder.addDoubleProperty("shoulder/error", () -> getError(toDegrees(shoulder.getEncoderVelocity()), toDegrees(getShoulderTarget())), null);
        builder.addDoubleProperty("shoulder/absolute", shoulderEncoder::getDegrees, null);

        // builder.addDoubleProperty("shoulder/shoulder-target-velo", () -> toDegrees(shoulderPID.getSetpoint().velocity), null);
        // builder.addDoubleProperty("shoulder/shoulder-target-pos", () -> toDegrees(shoulderPID.getSetpoint().position), null);

        // builder.addDoubleProperty("wrist/angle", () -> toDegrees(wrist.getEncoderPosition()), null);
        // builder.addDoubleProperty("wrist/velo", () -> toDegrees(wrist.getEncoderVelocity()), null);
        // builder.addDoubleProperty("wrist/target", () -> toDegrees(getWristTarget()), null);
        // builder.addDoubleProperty("wrist/error", () -> getError(toDegrees(wrist.getEncoderVelocity()), toDegrees(getWristTarget())), null);
        builder.addDoubleProperty("wrist/absolute", wristEncoder::getDegrees, null);

        // builder.addDoubleProperty("wrist/wrist-target-velo", () -> toDegrees(wristPID.getSetpoint().velocity), null);
        // builder.addDoubleProperty("wrist/wrist-target-pos", () -> toDegrees(wristPID.getSetpoint().position), null);

        builder.addDoubleProperty("armpose/x", () -> getArmPose().getX(), null);
        builder.addDoubleProperty("armpose/y", () -> getArmPose().getY(), null);
        builder.addDoubleProperty("armpose/z", () -> getArmPose().getZ(), null);
        builder.addDoubleProperty("armpose/roll", () -> -getArmPose().getRotation().getX(), null);
        builder.addDoubleProperty("armpose/pitch", () -> -getArmPose().getRotation().getY(), null);
        builder.addDoubleProperty("armpose/yaw", () -> getArmPose().getRotation().getZ(), null);
    }


}
