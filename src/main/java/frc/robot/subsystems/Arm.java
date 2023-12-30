package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIOReal;
import frc.robot.subsystems.claw.Claw.ClawState;
import frc.robot.util.kinematics.ArmKinematics;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.kinematics.ArmTrajectory;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;

import static com.ctre.phoenix6.signals.InvertedValue.*;
import static com.ctre.phoenix6.signals.NeutralModeValue.*;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.applyCurrentLimit;
import static frc.robot.subsystems.Arm.ArmMode.*;
import static java.lang.Math.*;

public class Arm extends StateMachine<Arm.ArmMode> {

    private final ArmKinematics kinematics = new ArmKinematics(baseToTurret, turretToShoulder, shoulderToWrist, wristToEndEffector);

    private final MotionMagicTalonFX elevator = new MotionMagicTalonFX(ELEVATOR_ID, ELEVATOR_GAINS, ELEVATOR_INPUT_TO_OUTPUT, ELEVATOR_MAX_VEL, ELEVATOR_MAX_ACCEL);

    //Shoulder hardware
    private final MotionMagicTalonFX shoulderLeader = new MotionMagicTalonFX(
            SHOULDER_LEADER_ID, SHOULDER_GAINS, SHOULDER_INPUT_TO_OUTPUT, SHOULDER_VEL, SHOULDER_ACCEL, SHOULDER_JERK
    );
    private final MotionMagicTalonFX shoulderFollower = new MotionMagicTalonFX(
            SHOULDER_FOLLOWER_ID, SHOULDER_GAINS, SHOULDER_INPUT_TO_OUTPUT, SHOULDER_VEL, SHOULDER_ACCEL, SHOULDER_JERK
    );
    private final ThroughBoreEncoder shoulderEncoder = new ThroughBoreEncoder(SHOULDER_ENCODER_PORT, SHOULDER_ENCODER_OFFSET);

    //Wrist hardware
    private final MotionMagicTalonFX wrist = new MotionMagicTalonFX(
            WRIST_ID, WRIST_GAINS, WRIST_INPUT_TO_OUTPUT, WRIST_VEL, WRIST_ACCEL, WRIST_JERK
    );
    private final ThroughBoreEncoder wristEncoder = new ThroughBoreEncoder(WRIST_ENCODER_PORT, WRIST_ENCODER_OFFSET);

    private final Claw claw = new Claw(new ClawIOReal());

    private ArmState currentArmState = STOWED_POS;


    public Arm() {
        super("Arm", UNDETERMINED, ArmMode.class);

        configureHardware();

        addChildSubsystem(claw);

        defineTransitions();
        registerStateCommands();

        //TODO: uncomment
        //goToArmState(STOWED_POS);
    }

    public Command openClaw() {
        return new InstantCommand(() -> {
            claw.requestTransition(ClawState.OPENED);
        });
    }

    public Command closeClaw() {
        return new InstantCommand(() -> {
            claw.requestTransition(ClawState.CLOSED);
        });
    }

    public ClawState getClawState() {
        return claw.getState();
    }

    public Command enableClawProx() {
        return new InstantCommand(claw::enableProx);
    }

    public Command disableClawProx() {
        return new InstantCommand(claw::disableProx);
    }

    private void defineTransitions() {
        addOmniTransition(SEEKING_STOWED);
//        addTransition(SEEKING_STOWED, STOWED, new InstantCommand(() -> {new WaitCommand(2).andThen(new InstantCommand(this::pullAbsoluteAngles)).schedule();}));
        addTransition(SEEKING_STOWED, STOWED);

        addOmniTransition(SOFT_STOP, () -> {
            elevator.set(0);
            shoulderLeader.set(0);
            wrist.set(0);
        });

        addTransition(STOWED, PRIMED, () -> goToArmState(PRIMED_POS));
        addOmniTransition(SEEKING_PRIMED);
        addTransition(SEEKING_PRIMED, PRIMED);
        addOmniTransition(SEEKING_PICKUP_GROUND);

        addTransition(PRIMED, NEW_GROUND_INTERMEDIATE, () -> goToArmState(NEW_INTERMEDIATE_GROUND_PICKUP_POS));
        addTransition(NEW_GROUND_INTERMEDIATE, NEW_GROUND_PICKUP, () -> goToArmState(NEW_GROUND_PICKUP_POS));
        addTransition(NEW_GROUND_PICKUP, LOW_SCORE, () -> goToArmState(LOW_POS));
        
        //New ground pickup stuff
        //Make sure we can go to and from low score (for auto)
        addTransition(LOW_SCORE, NEW_GROUND_PICKUP, () -> goToArmState(NEW_GROUND_PICKUP_POS));
        addTransition(LOW_SCORE, NEW_GROUND_INTERMEDIATE, () -> goToArmState(NEW_INTERMEDIATE_GROUND_PICKUP_POS));

        //Teleop stuff
        addTransition(STOWED, NEW_GROUND_PICKUP, () -> {
            goToArmState(NEW_GROUND_PICKUP_POS);
            // claw.disableProx();
            // setArmSlowSpeed();
        });
        addTransition(STOWED, TELEOP_GROUND_INTERMEDIATE, () -> {
            goToArmState(TELEOP_GROUND_INTERMEDIATE_POS);
            claw.disableProx();
            // setArmSlowSpeed();
        });

        addTransition(TELEOP_GROUND_INTERMEDIATE, NEW_GROUND_PICKUP, () -> goToArmState(NEW_GROUND_PICKUP_POS));


        //Make sure we can go from pickup to intermediate (for screwing around in the community)
        addTransition(NEW_GROUND_PICKUP, TELEOP_GROUND_INTERMEDIATE, () -> goToArmState(TELEOP_GROUND_INTERMEDIATE_POS));

        //Make sure the arm will go normal speed again when it exits these states
        addTransition(NEW_GROUND_PICKUP, SEEKING_STOWED, setArmNormalSpeedCommand().alongWith(enableClawProx()));
        addTransition(NEW_GROUND_INTERMEDIATE, SEEKING_STOWED, setArmNormalSpeedCommand().alongWith(enableClawProx()));
        
        addTransition(STOWED, LOW_SCORE, () -> goToArmState(LOW_POS));
        addTransition(PRIMED, LOW_SCORE, () -> goToArmState(LOW_POS));
        addTransition(STOWED, MID_SCORE, () -> goToArmState(MID_POS));
        addTransition(PRIMED, MID_SCORE, () -> goToArmState(MID_POS));
        addTransition(STOWED, HIGH_CUBE, () -> goToArmState(HIGH_CUBE_POS));
        addTransition(PRIMED, HIGH_CUBE, () -> goToArmState(HIGH_CUBE_POS));

        addTransition(STOWED, SEEKING_POSE);
        addTransition(SEEKING_POSE, AT_POSE);

        addTransition(PICKUP_GROUND, SEEKING_HIGH);

        addTransition(STOWED, SEEKING_PICKUP_DOUBLE, claw.transitionCommand(ClawState.OPENED));
        addTransition(SEEKING_PICKUP_DOUBLE, SEEKING_HIGH);

        addTransition(STOWED, SEEKING_HIGH);
        addTransition(PRIMED, SEEKING_HIGH);
        removeTransition(SEEKING_HIGH, STOWED);
        addTransition(SEEKING_HIGH, HIGH);
        addTransition(HIGH_CUBE, SEEKING_PICKUP_GROUND);

        addTransition(PICKUP_GROUND, SEEKING_STOWED, new InstantCommand(() -> setWristTarget(toRadians(-45))));
        addTransition(HIGH, SEEKING_STOWED, () -> setWristTarget(toRadians(-45)));

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
                    setShoulderTarget(toRadians(45));
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
//                setRotatorTarget(currentArmState.getRotatorAngle());
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
        NEW_GROUND_PICKUP, NEW_GROUND_INTERMEDIATE, TELEOP_GROUND_INTERMEDIATE,
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

        shoulderLeader.configure(Brake, Clockwise_Positive);
        applyCurrentLimit(shoulderLeader);

        shoulderFollower.configure(Brake, Clockwise_Positive);
        applyCurrentLimit(shoulderFollower);
        
        wrist.configure(Brake, CounterClockwise_Positive);
        applyCurrentLimit(wrist);
        
        new WaitCommand(2)
        .andThen(setShoulderFollower())
        .schedule();
    }

    public Command setShoulderFollower() {
        return new WhileDisabledInstantCommand(() -> shoulderFollower.setControl(new Follower(shoulderLeader.getDeviceID(), false)));
    }

    public Command calculateElevatorFF(Trigger increment, BooleanSupplier interrupt) {
        return elevator.calculateKV(ELEVATOR_GAINS.getS(), 0.05, increment, interrupt);
    }

    public Command calculateWristFF(Trigger increment, BooleanSupplier interrupt) {
        return wrist.calculateKV(WRIST_GAINS.getS(), 0.05, increment, interrupt);
    }


    public Command calculateShoulderFF(Trigger increment, BooleanSupplier interrupt) {
        return shoulderLeader.calculateKV(SHOULDER_GAINS.getS(), 0.05, increment, interrupt);
    }

    public InstantCommand reset() {
        return new InstantCommand(this::pollAbsoluteAngles);
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
        shoulderLeader.setTarget(target);
    }

    /**
     * Set the target of the wrist
     * @param target target angle (in radians)
     */
    public void setWristTarget(double target) {
        wrist.setTarget(target);
    }


    @Override
    protected void determineSelf() {
        pollAbsoluteAngles();

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
//        wristPID.reset(getWristAngle());
//        shoulderPID.reset(getShoulderAngle());

        setElevatorTarget(getElevatorHeight());

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

    public Claw claw() {return claw;}

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
        return shoulderLeader.getTarget();
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

    public boolean isShoulderSafe() {
        return getElevatorHeight() <= ELEVATOR_TOLERANCE;
    }

    public boolean isElevatorSafe() {
        return getShoulderAngle() <= SHOULDER_ELEVATOR_THRESHOLD;
    }

    public void pollAbsoluteAngles() {
        shoulderLeader.resetPosition(shoulderEncoder.getRadians());

        wrist.resetPosition(wristEncoder.getRadians());
    }

    public void setArmSlowSpeed() {
        shoulderLeader.changeSpeed(SHOULDER_SLOW_VEL, SHOULDER_SLOW_ACCEL, SHOULDER_SLOW_JERK);
        wrist.changeSpeed(WRIST_SLOW_VEL, WRIST_SLOW_ACCEL, WRIST_SLOW_JERK);
    }

    public Command setArmSlowSpeedCommand() {
        return new InstantCommand(this::setArmSlowSpeed);
    }

    public void setArmNormalSpeed() {
        shoulderLeader.changeSpeed(SHOULDER_VEL, SHOULDER_ACCEL, SHOULDER_JERK);
        wrist.changeSpeed(WRIST_VEL, WRIST_ACCEL, WRIST_JERK);
    }

    public Command setArmNormalSpeedCommand() {
        return new InstantCommand(this::setArmNormalSpeed);
    }

    public void setArmFastSpeed() {
        shoulderLeader.changeSpeed(SHOULDER_FAST_VEL, SHOULDER_FAST_ACCEL, SHOULDER_FAST_JERK);
        wrist.changeSpeed(WRIST_VEL, WRIST_ACCEL, WRIST_JERK);
    }

    public Command setArmFastSpeedCommand() {
        return new InstantCommand(this::setArmFastSpeed);
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {

        // builder.addDoubleProperty("elevator/output", () -> Units.metersToInches(getElevatorHeight()), null);
        builder.addDoubleProperty("elevator/height", () -> Units.metersToInches(getElevatorHeight()), null);
        builder.addDoubleProperty("elevator/target", () -> Units.metersToInches(getElevatorTarget()), null);
        // builder.addDoubleProperty("elevator/error", () -> getError(Units.metersToInches(getElevatorTarget()), Units.metersToInches(getElevatorHeight())), null);

        // builder.addDoubleProperty("shoulder/velo", () -> toDegrees(shoulder.getEncoderVelocity()), null);
        builder.addDoubleProperty("shoulder/motor-angle", () -> toDegrees(shoulderLeader.getEncoderPosition()), null);
        builder.addDoubleProperty("shoulder/target", () -> toDegrees(getShoulderTarget()), null);
        builder.addDoubleProperty("shoulder/absolute-angle", shoulderEncoder::getDegrees, null);

        builder.addDoubleProperty("shoulder/absolute-error", () -> getError(shoulderEncoder.getDegrees(), toDegrees(getShoulderTarget())), null);
        builder.addDoubleProperty("shoulder/motor-error", () -> getError(toDegrees(shoulderLeader.getEncoderPosition()), toDegrees(getShoulderTarget())), null);

        // builder.addDoubleProperty("shoulder/shoulder-target-velo", () -> toDegrees(shoulderPID.getSetpoint().velocity), null);
        // builder.addDoubleProperty("shoulder/shoulder-target-pos", () -> toDegrees(shoulderPID.getSetpoint().position), null);

        // builder.addDoubleProperty("wrist/velocity_error", () -> toDegrees(wristPID.getVelocityError()), null);
        // builder.addDoubleProperty("wrist/position_error", () -> toDegrees(wristPID.getPositionError()), null);
        builder.addDoubleProperty("wrist/absolute-angle", wristEncoder::getDegrees, null);
        builder.addDoubleProperty("wrist/motor-angle", () -> toDegrees(wrist.getEncoderPosition()), null);
        builder.addDoubleProperty("wrist/target", () -> toDegrees(getWristTarget()), null);
        // builder.addDoubleProperty("wrist/velo", () -> toDegrees(wrist.getEncoderVelocity()), null);

        builder.addDoubleProperty("wrist/absolute-error", () -> getError(wristEncoder.getDegrees(), toDegrees(getWristTarget())), null);
        builder.addDoubleProperty("wrist/motor-error", () -> getError(toDegrees(wrist.getEncoderPosition()), toDegrees(getWristTarget())), null);

    }


}
