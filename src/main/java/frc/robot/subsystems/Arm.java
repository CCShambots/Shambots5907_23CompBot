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
import frc.robot.ShamLib.motors.pro.EnhancedTalonFXPro;
import frc.robot.ShamLib.motors.pro.MotionMagicTalonFXPro;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.subsystems.Claw.State;
import frc.robot.util.kinematics.ArmKinematics;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.kinematics.ArmTrajectory;

import java.util.function.BooleanSupplier;

import static com.ctre.phoenixpro.signals.InvertedValue.*;
import static com.ctre.phoenixpro.signals.NeutralModeValue.*;
import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.applyCurrentLimit;
import static frc.robot.subsystems.Arm.ArmMode.*;
import static java.lang.Math.*;

public class Arm extends StateMachine<Arm.ArmMode> {

    private final ArmKinematics kinematics = new ArmKinematics(baseToTurret, turretToShoulder, shoulderToWrist, wristToEndEffector);

    private final MotionMagicTalonFXPro elevator = new MotionMagicTalonFXPro(ELEVATOR_ID, ELEVATOR_GAINS, ELEVATOR_INPUT_TO_OUTPUT, ELEVATOR_MAX_VEL, ELEVATOR_MAX_ACCEL);

    //Shoulder hardware
    private final EnhancedTalonFXPro shoulder = new EnhancedTalonFXPro(SHOULDER_ID, SHOULDER_INPUT_TO_OUTPUT);/*new VelocityTalonFXPro(SHOULDER_ID, SHOULDER_GAINS, SHOULDER_INPUT_TO_OUTPUT);*/
    private final ThroughBoreEncoder shoulderEncoder = new ThroughBoreEncoder(SHOULDER_ENCODER_PORT, SHOULDER_ENCODER_OFFSET);
    
    //Shoulder control loops
    private final ProfiledPIDController shoulderPID = new ProfiledPIDController(SHOULDER_GAINS.p, SHOULDER_GAINS.i, SHOULDER_GAINS.d,
        new TrapezoidProfile.Constraints(SHOULDER_VEL, SHOULDER_ACCEL), 0.005);
    private final ArmFeedforward shoulderFF = new ArmFeedforward(SHOULDER_KS, SHOULDER_KG, SHOULDER_KV);
    private double shoulderTarget = toRadians(0);
    
    //Wrist hardware
    private final EnhancedTalonFXPro wrist = new EnhancedTalonFXPro(WRIST_ID, WRIST_INPUT_TO_OUTPUT);
    private final ThroughBoreEncoder wristEncoder = new ThroughBoreEncoder(WRIST_ENCODER_PORT, WRIST_ENCODER_OFFSET);

    //Wrist control loops
    private final ArmFeedforward wristFF = new ArmFeedforward(WRIST_KS, WRIST_KG, WRIST_KV);
    private final ProfiledPIDController wristPID = new ProfiledPIDController(WRIST_GAINS.p, WRIST_GAINS.i, WRIST_GAINS.d,
        new TrapezoidProfile.Constraints(WRIST_VEL, WRIST_ACCEL), 0.005);
    private double wristTarget = toRadians(0);

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

        //TODO: uncomment
        //goToArmState(STOWED_POS);
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

    public Command enableClawProx() {
        return new InstantCommand(claw::enableProx);
    }

    public Command disableClawProx() {
        return new InstantCommand(claw::disableProx);
    }
    private void defineTransitions() {
        addOmniTransition(SEEKING_STOWED);
        addTransition(SEEKING_STOWED, STOWED);

        addOmniTransition(SOFT_STOP, () -> {
            elevator.set(0);
            shoulder.set(0);
            wrist.set(0);
        });

        addTransition(STOWED, PRIMED, () -> goToArmState(PRIMED_POS));
        addOmniTransition(SEEKING_PRIMED);
        addTransition(SEEKING_PRIMED, PRIMED);
        addOmniTransition(SEEKING_PICKUP_GROUND);

        addTransition(STOWED, LOW_SCORE, () -> goToArmState(LOW_POS));
        addTransition(PRIMED, LOW_SCORE, () -> goToArmState(LOW_POS));
        addTransition(STOWED, MID_SCORE, () -> goToArmState(MID_POS));
        addTransition(PRIMED, MID_SCORE, () -> goToArmState(MID_POS));
        addTransition(STOWED, HIGH_CUBE, () -> goToArmState(HIGH_CUBE_POS));
        addTransition(PRIMED, HIGH_CUBE, () -> goToArmState(HIGH_CUBE_POS));

        addTransition(STOWED, SEEKING_POSE);
        addTransition(SEEKING_POSE, AT_POSE);

        addTransition(STOWED, SEEKING_PICKUP_DOUBLE, claw.transitionCommand(State.OPENED));
        addTransition(SEEKING_PICKUP_DOUBLE, SEEKING_HIGH);

        addTransition(STOWED, SEEKING_HIGH);
        addTransition(PRIMED, SEEKING_HIGH);
        removeTransition(SEEKING_HIGH, STOWED);
        addTransition(SEEKING_HIGH, HIGH);
        addTransition(HIGH_CUBE, SEEKING_PICKUP_GROUND);

        addTransition(PICKUP_GROUND, SEEKING_STOWED, new InstantCommand(() -> setWristTarget(toRadians(-45))));

        addTransition(STOWED, SEEKING_PICKUP_GROUND);
        addTransition(HIGH, SEEKING_PICKUP_GROUND);
        removeTransition(SEEKING_PICKUP_GROUND, STOWED);
        addTransition(SEEKING_PICKUP_GROUND, PICKUP_GROUND);
        addTransition(HIGH_CUBE, SEEKING_PICKUP_GROUND);
    }

    private void registerStateCommands() {
        registerStateCommand(SEEKING_HIGH, 
            new ExtendArmCommand(this, HIGH_POS).andThen(transitionCommand(HIGH))
        );

        registerStateCommand(SEEKING_PICKUP_GROUND, 
            new ExtendArmCommand(this, GROUND_PICKUP_POS).andThen(transitionCommand(PICKUP_GROUND))
        );

        registerStateCommand(SEEKING_PICKUP_DOUBLE, 
            new ExtendArmCommand(this, PICKUP_DOUBLE_POS).andThen(transitionCommand(PICKUP_DOUBLE))
        );

        registerStateCommand(SEEKING_STOWED,
        new FunctionalCommand(() -> {
            if(getWristAngle() < 0 && getShoulderAngle() < toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT)) setWristTarget(0);
            if(getShoulderAngle() < 0) setShoulderTarget(toRadians(15));
        }, () -> {}, (interrupted) -> {}, () -> getShoulderAngle() >=0 && (getShoulderAngle() >= toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT) || getWristAngle() >=0 )).andThen(
            new FunctionalCommand(
                () -> {
                    setElevatorTarget(STOWED_POS.getElevatorExtension());
                    setShoulderTarget(toRadians(-45));
                },
                () -> {},
                (interrupted) -> {},
                () -> getError(getElevatorHeight(), getElevatorTarget()) <= ELEVATOR_TOLERANCE
            ),
            new InstantCommand(() -> goToArmState(STOWED_POS)),
            new InstantCommand(() -> requestTransition(STOWED))
        )
        );

        registerStateCommand(SEEKING_PRIMED,
        new FunctionalCommand(() -> {
            if(getWristAngle() < 0 && getShoulderAngle() < toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT)) setWristTarget(0);
            if(getShoulderAngle() < 0) setShoulderTarget(toRadians(15));
        }, () -> {}, (interrupted) -> {}, () -> getShoulderAngle() >=0 && (getShoulderAngle() >= toRadians(SHOULDER_REQUIRED_STOWED_HEIGHT) || getWristAngle() >=0 )).andThen(
            new FunctionalCommand(
                () -> {
                    setElevatorTarget(PRIMED_POS.getElevatorExtension());
                    setShoulderTarget(toRadians(-45));
                },
                () -> {},
                (interrupted) -> {},
                () -> getError(getElevatorHeight(), getElevatorTarget()) <= ELEVATOR_TOLERANCE
            ),
            new InstantCommand(() -> goToArmState(PRIMED_POS)),
            new InstantCommand(() -> requestTransition(PRIMED))
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
    }


    public enum ArmMode {
        UNDETERMINED, 
        SEEKING_STOWED, STOWED,
        SEEKING_PICKUP_DOUBLE, PICKUP_DOUBLE,
        SEEKING_PICKUP_GROUND, PICKUP_GROUND,
        LOW_SCORE, MID_SCORE,
        SEEKING_HIGH, HIGH,
        HIGH_CUBE,
        SEEKING_PRIMED, PRIMED,
        SOFT_STOP,

        SEEKING_POSE,
        AT_POSE,

        TESTING
    }

    private void configureHardware() {
        shoulderEncoder.setInverted(false);
        wristEncoder.setInverted(true);


        elevator.configure(Brake, CounterClockwise_Positive);
        applyCurrentLimit(elevator);

        shoulder.configure(Brake, CounterClockwise_Positive);
        applyCurrentLimit(shoulder);

        wrist.configure(Brake, CounterClockwise_Positive);
        applyCurrentLimit(wrist);

    }

    public Command calculateElevatorFF(Trigger increment, BooleanSupplier interrupt) {
        return elevator.calculateKV(ELEVATOR_GAINS.getS(), 0.05, increment, interrupt);
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
            if (isEnabled()) {
                //Wrist code
                double wristPIDOutput = wristPID.calculate(wristEncoder.getRadians(), wristTarget);
                double wristFFOutput = wristFF.calculate(wristPID.getSetpoint().position + shoulderEncoder.getRadians(), wristPID.getSetpoint().velocity);

                
                wrist.setVoltage(wristPIDOutput + wristFFOutput);
                
                //Shoulder code
                double shoulderPIDOutput = shoulderPID.calculate(getShoulderAngle(), shoulderTarget);
                double shoulderFFOutput = shoulderFF.calculate(shoulderPID.getSetpoint().position,  shoulderPID.getSetpoint().velocity);
                
                shoulder.setVoltage(shoulderPIDOutput + shoulderFFOutput);
            }
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
        wristPID.reset(getWristAngle());
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

        setArmNormalSpeed();

        setState(STOWED);
        requestTransition(ArmMode.SEEKING_STOWED);
    }

    @Override
    protected void update() {
    }

    @Override
    protected void onEnable() {
        //Make sure no I buildup or anything insane happens
        wristPID.reset(getWristAngle());
        shoulderPID.reset(getShoulderAngle());

        setArmNormalSpeed();
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
        return shoulderPID.getGoal().position;
    }

    public double getWristAngle() {
        return wristEncoder.getRadians();
    }

    public double getWristTarget() {
        return wristPID.getGoal().position;
    }

    private double getError(double num1, double num2) {
        return abs(num1 - num2);
    }

    public boolean isShoulderSafe() {
        return getElevatorHeight() <= ELEVATOR_TOLERANCE;
    }

    public boolean isElevatorSafe() {
        return getShoulderAngle() <= SHOULDER_ELEVATOR_THRESHOLD;
    }

    public void pullAbsoluteAngles() {
        shoulder.resetPosition(shoulderEncoder.getRadians());
        wrist.resetPosition(wristEncoder.getRadians());
    }

    public void setArmSlowSpeed() {
        shoulderPID.setConstraints(new TrapezoidProfile.Constraints(SHOULDER_SLOW_VEL, SHOULDER_SLOW_ACCEL));
        wristPID.setConstraints(new TrapezoidProfile.Constraints(WRIST_SLOW_VEL, WRIST_SLOW_ACCEL));
    }

    public Command setArmSlowSpeedCommand() {
        return new InstantCommand(this::setArmSlowSpeed);
    }

    public void setArmNormalSpeed() {
        shoulderPID.setConstraints(new TrapezoidProfile.Constraints(SHOULDER_VEL, SHOULDER_ACCEL));
        wristPID.setConstraints(new TrapezoidProfile.Constraints(WRIST_VEL, WRIST_ACCEL));
    }

    public Command setArmNormalSpeedCommand() {
        return new InstantCommand(this::setArmNormalSpeed);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

        // builder.addDoubleProperty("elevator/output", () -> Units.metersToInches(getElevatorHeight()), null);
        builder.addDoubleProperty("elevator/height", () -> Units.metersToInches(getElevatorHeight()), null);
        // builder.addDoubleProperty("elevator/target", () -> Units.metersToInches(getElevatorTarget()), null);
        // builder.addDoubleProperty("elevator/error", () -> getError(Units.metersToInches(getElevatorTarget()), Units.metersToInches(getElevatorHeight())), null);

        builder.addDoubleProperty("shoulder/velo", () -> toDegrees(shoulder.getEncoderVelocity()), null);
        // builder.addDoubleProperty("shoulder/angle", () -> toDegrees(shoulder.getEncoderPosition()), null);
        // builder.addDoubleProperty("shoulder/target", () -> toDegrees(getShoulderTarget()), null);
        // builder.addDoubleProperty("shoulder/error", () -> getError(toDegrees(shoulder.getEncoderVelocity()), toDegrees(getShoulderTarget())), null);
        builder.addDoubleProperty("shoulder/absolute", shoulderEncoder::getDegrees, null);

        builder.addDoubleProperty("shoulder/velocity_error", () -> toDegrees(shoulderPID.getVelocityError()), null);
        builder.addDoubleProperty("shoulder/position_error", () -> toDegrees(shoulderPID.getPositionError()), null);
        builder.addDoubleProperty("shoulder/absolute", shoulderEncoder::getDegrees, null);

        builder.addDoubleProperty("shoulder/shoulder-target-velo", () -> toDegrees(shoulderPID.getSetpoint().velocity), null);
        builder.addDoubleProperty("shoulder/shoulder-target-pos", () -> toDegrees(shoulderPID.getSetpoint().position), null);

        builder.addDoubleProperty("wrist/velocity_error", () -> toDegrees(wristPID.getVelocityError()), null);
        builder.addDoubleProperty("wrist/position_error", () -> toDegrees(wristPID.getPositionError()), null);
        builder.addDoubleProperty("wrist/absolute", wristEncoder::getDegrees, null);
        builder.addDoubleProperty("wrist/velo", () -> toDegrees(wrist.getEncoderVelocity()), null);

        builder.addDoubleProperty("wrist/wrist-target-velo", () -> toDegrees(wristPID.getSetpoint().velocity), null);
        builder.addDoubleProperty("wrist/wrist-target-pos", () -> toDegrees(wristPID.getSetpoint().position), null);

        builder.addDoubleProperty("armpose/x", () -> getArmPose().getX(), null);
        builder.addDoubleProperty("armpose/y", () -> getArmPose().getY(), null);
        builder.addDoubleProperty("armpose/z", () -> getArmPose().getZ(), null);
        builder.addDoubleProperty("armpose/roll", () -> -getArmPose().getRotation().getX(), null);
        builder.addDoubleProperty("armpose/pitch", () -> -getArmPose().getRotation().getY(), null);
        builder.addDoubleProperty("armpose/yaw", () -> getArmPose().getRotation().getZ(), null);
    }


}
