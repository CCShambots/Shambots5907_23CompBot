package frc.robot.util.kinematics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import static java.lang.Math.round;

public class ArmTrajectory {

    private final ArmKinematics kinematics;
    private final double time;

    private final Map<Integer, ArmState> states = new HashMap<>(); //List of states at various steps along the trajectory (in 20 ms intervals)


    public ArmTrajectory(ArmKinematics kinematics, ArmState currentArmState, Pose3d endPose, double time) {
        this.kinematics = kinematics;
        this.time = time;

        Pose3d startPose = kinematics.forwardKinematics(currentArmState);

        Transform3d totalTransform = new Transform3d(startPose, endPose);

        double steps = time * 50;
        Transform3d perStepTransform = totalTransform.div(steps);

        Pose3d currentPose = startPose;
        ArmState lastState = kinematics.chooseIdealState(currentArmState, kinematics.inverseKinematics(startPose));

        for(int i = 0; i<steps; i++) {
            currentPose = currentPose.transformBy(perStepTransform);
            ArmState currentState = kinematics.chooseIdealState(lastState, kinematics.inverseKinematics(currentPose));
            states.put(i, currentState);
            lastState = currentState;
        }
    }

    public ArmState get(double seconds) {
        double ms = seconds * 1000;
        int loops = (int) round(ms / 20);

        if(loops < states.size() - 1) {
            return states.get(loops);
        } else {
            return states.get(states.size()-1);
        }
    }

    public Command run(Consumer<ArmState> armStateConsumer, Subsystem... requirements) {
        Timer timer = new Timer();

        return new FunctionalCommand(
                () -> timer.start(),
                () -> {
                    armStateConsumer.accept(get(timer.get()));
                },
                (interrupted) -> {

                },
                () -> timer.get() > time,
                requirements
        );
    }


}
