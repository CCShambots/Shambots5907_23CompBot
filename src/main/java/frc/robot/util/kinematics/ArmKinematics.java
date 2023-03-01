package frc.robot.util.kinematics;

import edu.wpi.first.math.geometry.*;

import static java.lang.Math.*;

public class ArmKinematics {

    private double baseToTurret;
    private double turretToArm;
    private double armToWrist;

    private double wristToEndEffector;

    public ArmKinematics(double baseToTurret, double elevatorToArm, double armToWrist, double wristToEndEffector) {
        this.baseToTurret = baseToTurret;
        this.turretToArm = elevatorToArm;
        this.armToWrist = armToWrist;
        this.wristToEndEffector = wristToEndEffector;
    }

    public Pose3d forwardKinematics(ArmState state) {
        Pose3d workingPose = new Pose3d();

        //Turret pose
        workingPose = workingPose.transformBy(new Transform3d(new Translation3d(0, 0, baseToTurret), new Rotation3d(0, 0, state.getTurretAngle())));

        //Shoulder pose
        workingPose = workingPose.transformBy(new Transform3d(new Translation3d(0, 0, turretToArm + state.getElevatorExtension()), new Rotation3d(0, 0, 0)));

        //Wrist pose
        workingPose = workingPose.transformBy(new Transform3d(new Translation3d(armToWrist, new Rotation3d(0, -state.getShoulderAngle(), 0)), new Rotation3d(0, -state.getShoulderAngle(), 0)));

        //EE pose (no rotator
        workingPose = workingPose.transformBy(new Transform3d(new Translation3d(wristToEndEffector, new Rotation3d(0, -(state.getWristAngle()), 0)), new Rotation3d(0, -state.getWristAngle(), 0)));

        //Apply rotator
        workingPose = workingPose.transformBy(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(-state.getRotatorAngle(), 0, 0)));

        return workingPose;

    }

    public ArmState[] inverseKinematics(Pose3d targetPose) {
        double x = targetPose.getX();
        double y = targetPose.getY();
        double z = targetPose.getZ();

        double R = targetPose.getRotation().getX();
        double P = targetPose.getRotation().getY();

        double b = baseToTurret;
        double t = turretToArm;
        double a = armToWrist;
        double w = wristToEndEffector;

        //Get turret angle
        //This could be done from the yaw of the pose, but I think that's dumb because they shouldn't have to calculate turret angle manually
        double theta = atan2(y, x);

        //Get radius of position
        double r = hypot(x, y);

        //Extract 2d pose of the arm after turret rotation
        Pose2d pose2d = new Pose2d(r, z, new Rotation2d(-P));

        //Extract the wrist pose of the wrist joint in 2d space
        Pose2d pose2d2 = pose2d.transformBy(new Transform2d(new Translation2d(-w, new Rotation2d(0)), new Rotation2d(0, 0)));

        //Determine the height of the triangle formed by arm joint, wrist, joint, and intersection of those two to the elevator.
        double h = sqrt(pow(a, 2) - pow(pose2d2.getX(), 2));

        //Extract the elevator extension
        double EPos = pose2d2.getY() - b - t - h;
        double ENeg = pose2d2.getY() - b - t + h;

        //Extract the arm angle
        double alphaPos = asin(h/a);
        double alphaNeg = asin(-h/a);

        //Extract the wrist angle
        double betaPos = -P - alphaPos;
        double betaNeg = -P - alphaNeg;

        //Extract the rotator angle
        double delta = -R;

        return new ArmState[]{
                new ArmState(theta, EPos, alphaPos, betaPos, delta),
                new ArmState(theta, ENeg, alphaNeg, betaNeg, delta),
        };
    }

    /**
     * Return the best arm state. This will check if the states are valid (i.e. if they don't
     * @param currentState the current state of the arm
     * @param options the potential solutions
     * @return the ideal state, if there is one
     */
    public ArmState chooseIdealState(ArmState currentState, ArmState... options) {
        double bestDistance = Double.MAX_VALUE;
        int bestIndex = -1;
        for(int i = 0; i<options.length; i++) {
            ArmState test = options[i];
            double currentEDistance = abs(currentState.getElevatorExtension() - test.getElevatorExtension());
            if(currentEDistance < bestDistance && test.isValid()) {
                bestIndex = i;
                bestDistance = currentEDistance;
            }
        }

        return bestIndex != -1 ? options[bestIndex] : null;
    }

    public void printPose(Pose3d pose) {
        System.out.println("x: " + pose.getX() + ", y: " + pose.getY() + ", z:" + pose.getZ() + "Rotation: (" + pose.getRotation().getX() + ", " + pose.getRotation().getY() + ", " + pose.getRotation().getZ() + ")");
    }

    

}
