package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.v5.MotionMagicTalonFXV5;
import frc.robot.ShamLib.motors.rev.PositionSpark;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;
import frc.robot.subsystems.ClawVision.VisionState;
import frc.robot.util.kinematics.ArmKinematics;
import frc.robot.util.kinematics.ArmState;
import frc.robot.util.kinematics.ArmTrajectory;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Arm.*;
import static frc.robot.subsystems.Arm.ArmMode.*;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Arm extends StateMachine<Arm.ArmMode> {

    private final ArmKinematics kinematics = new ArmKinematics(baseToTurret, turretToArm, armToWrist, wristToEndEffector);

    //TODO: Make [-180, 180]
    private final PositionSpark rotator = new PositionSpark(ROTATOR_ID, kBrushless, ROTATOR_PIDF_GAINS, ROTATOR_ENCODER_OFFSET, Math.toRadians(1));

    private final MotionMagicTalonFXV5 turret = new MotionMagicTalonFXV5(TURRET_ID, TURRET_GAINS, TURRET_INPUT_TO_OUTPUT);
    private final ThroughBoreEncoder turretEncoder = new ThroughBoreEncoder(TURRET_ENCODER_PORT, TURRET_ENCODER_OFFSET);

    private final MotionMagicTalonFXV5 elevator = new MotionMagicTalonFXV5(ELEVATOR_ID, ELEVATOR_GAINS, ELEVATOR_INPUT_TO_OUTPUT, ELEVATOR_MAX_VEL, ELEVATOR_MAX_ACCEL);
    private final WPI_TalonFX elevatorFollower = new WPI_TalonFX(ELEVATOR_FOLLOWER_ID);


    private final MotionMagicTalonFXV5 shoulder = new MotionMagicTalonFXV5(SHOULDER_ID, SHOULDER_GAINS, SHOULDER_INPUT_TO_OUTPUT, SHOULDER_MAX_VEL, SHOULDER_MAX_ACCEL);
    private final ThroughBoreEncoder shoulderEncoder = new ThroughBoreEncoder(SHOULDER_ENCODER_PORT, SHOULDER_ENCODER_OFFSET);

    private final MotionMagicTalonFXV5 wrist = new MotionMagicTalonFXV5(WRIST_ID, WRIST_GAINS, WRIST_INPUT_TO_OUTPUT, WRIST_MAX_VEL, WRIST_MAX_ACCEL);
    private final ThroughBoreEncoder wristEncoder = new ThroughBoreEncoder(WRIST_ENCODER_PORT, WRIST_ENCODER_OFFSET);

    private final ClawVision clawVision = new ClawVision();


    public Arm() {
        super("Arm", Undetermined, ArmMode.class);

        shoulderEncoder.setInverted(true);

        elevatorFollower.configFactoryDefault();
        elevatorFollower.setNeutralMode(NeutralMode.Brake);
        elevatorFollower.follow(elevator);

        elevator.setNeutralMode(NeutralMode.Brake);
        elevator.setInverted(true);

        elevatorFollower.setInverted(InvertType.FollowMaster);
        shoulder.setNeutralMode(NeutralMode.Coast);
        shoulder.setInverted(true);
        wrist.setNeutralMode(NeutralMode.Coast);

        addChildSubsystem(clawVision);

        addOmniTransition(TrackCone, new InstantCommand(() -> clawVision.requestTransition(VisionState.ConeAngle)));
        addOmniTransition(Idle, new InstantCommand());

        //TODO: Bring this into a separate command
        LinearFilter filter = LinearFilter.singlePoleIIR(.3, 0.02);
        registerStateCommand(TrackCone, new RunCommand(() -> {
            double remainder = Math.IEEEremainder(clawVision.getConeAngle().getRadians() + Math.PI/2, Math.PI);


            double offset = Math.IEEEremainder(remainder + rotator.getPosition(), 2*Math.PI);

            rotator.setTarget(filter.calculate(offset));
        }).andThen(new InstantCommand(() -> requestTransition(Idle))));



        //TODO: check if abs encoders are zero and disable joint on startup if so
    }

    public Command rotator2() {
        return new InstantCommand(() -> rotator.setTarget(Math.PI/2));
    }

    public Command rotator3() {
        return new InstantCommand(() -> {
            rotator.setTarget(-Math.PI/2);
        });
    }

    public Command calculateFF(double power) {
        return shoulder.calculateKF(power);
    }

    public InstantCommand shoulderUp() {
        return new InstantCommand(() -> {
            shoulder.setTarget(Math.toRadians(30));
        });
    }

    public InstantCommand shoulderDown() {
        return new InstantCommand(() -> {
            shoulder.setTarget(Math.toRadians(0));
        });
    }

    public InstantCommand reset() {
        return new InstantCommand(this::pullAbsoluteAngles);
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
            // goToPose(new Pose3d(
            //     .85,
            //     0,
            //     0.8,
            //     new Rotation3d(0, 0, 0)
            // ));
            goToPose(new Pose3d(
                    .56,
                    0,
                    0.96,
                    new Rotation3d(0, -Math.PI/2, 0)
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
            turret.setTarget(state.getTurretAngle());
            elevator.setTarget(state.getElevatorExtension());
            shoulder.setTarget(state.getShoulderAngle());
            wrist.setTarget(state.getWristAngle());
            rotator.setTarget(state.getRotatorAngle());
        }
    }

    public void goToPose(Pose3d pose) {
        ArmState target = runIK(pose);
        if(target != null) {
            pullAbsoluteAngles(); //TODO: Remove when no longer necessary
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

    @Override
    protected void determineSelf() {
        pullAbsoluteAngles();

        setState(Idle);
    }

    @Override
    protected void update() {
        // pullAbsoluteAngles();

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
        return turret.getPosition();
    }

    public double getTurretTarget() {
        return turret.getTarget();
    }

    public double getElevatorHeight() {
        return elevator.getPosition();
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
        turret.resetPosition(turretEncoder.getRadians());
        shoulder.resetPosition(shoulderEncoder.getRadians());
        wrist.resetPosition(wristEncoder.getRadians());
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleProperty("turret/angle", () -> toDegrees(getTurretAngle()), null);
        builder.addDoubleProperty("turret/target", () -> toDegrees(getTurretTarget()), null);
        builder.addDoubleProperty("turret/error", () -> getError(toDegrees(getTurretTarget()), toDegrees(getTurretAngle())), null);

        builder.addDoubleProperty("elevator/height", () -> Units.metersToInches(getElevatorHeight()), null);
        builder.addDoubleProperty("elevator/target", () -> Units.metersToInches(getElevatorTarget()), null);
        builder.addDoubleProperty("elevator/error", () -> getError(Units.metersToInches(getElevatorTarget()), Units.metersToInches(getElevatorHeight())), null);

        builder.addDoubleProperty("shoulder/angle", () -> toDegrees(shoulder.getPosition()), null);
        builder.addDoubleProperty("shoulder/target", () -> toDegrees(getShoulderTarget()), null);
        builder.addDoubleProperty("shoulder/error", () -> getError(toDegrees(shoulder.getPosition()), toDegrees(getShoulderTarget())), null);
        builder.addDoubleProperty("shoulder/absolute", () -> shoulderEncoder.getDegrees(), null);

        builder.addDoubleProperty("shoulder/power", () -> shoulder.getMotorOutputPercent(), null);

        builder.addDoubleProperty("wrist/angle", () -> toDegrees(wrist.getPosition()), null);
        builder.addDoubleProperty("wrist/target", () -> toDegrees(getWristTarget()), null);
        builder.addDoubleProperty("wrist/error", () -> getError(toDegrees(wrist.getPosition()), toDegrees(getWristTarget())), null);
        builder.addDoubleProperty("wrist/absolute", () -> wristEncoder.getDegrees(), null);

        builder.addDoubleProperty("wrist/raw-rot", () -> wrist.getSelectedSensorPosition() / 2048, null);
        builder.addDoubleProperty("wrist/output-rot", () -> wrist.getSelectedSensorPosition() / 2048 * (1/100.0), null);

        builder.addDoubleProperty("rotator/output", () -> rotator.getAppliedOutput(), null);

        builder.addDoubleProperty("rotator/angle", () -> toDegrees(getRotatorAngle()), null);
        builder.addDoubleProperty("rotator/target", () -> toDegrees(getRotatorTarget()), null);
        builder.addDoubleProperty("rotator/error", () -> getError(toDegrees(getRotatorTarget()), toDegrees(getRotatorAngle())), null);

        builder.addDoubleProperty("armpose/x", () -> getArmPose().getX(), null);
        builder.addDoubleProperty("armpose/y", () -> getArmPose().getY(), null);
        builder.addDoubleProperty("armpose/z", () -> getArmPose().getZ(), null);
        builder.addDoubleProperty("armpose/roll", () -> {return -getArmPose().getRotation().getX();}, null);
        builder.addDoubleProperty("armpose/pitch", () -> {return -getArmPose().getRotation().getY();}, null);
        builder.addDoubleProperty("armpose/yaw", () -> {return getArmPose().getRotation().getZ();}, null);
    }

    public enum ArmMode {
        Undetermined, Idle, TrackCone
    }


}
