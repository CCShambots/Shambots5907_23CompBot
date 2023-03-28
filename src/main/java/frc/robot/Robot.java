package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;


public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private final EventLoop checkModulesLoop = new EventLoop();

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer(checkModulesLoop);

    SubsystemManagerFactory.getInstance().registerSubsystem(robotContainer);
    SubsystemManagerFactory.getInstance().disableAllSubsystems();

    PathPlannerServer.startServer(5811); //TODO: disable before comp

    //Run the module control loops every 5 ms
    addPeriodic(robotContainer.runArmControlLoops(), 0.005);

    //Check the alliance from FMS when the bot turns on
    Constants.pullAllianceFromFMS(robotContainer);

    //Update the event loop for misaligned modules once every 10 seconds
    addPeriodic(checkModulesLoop::poll, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //TODO: Remove
    robotContainer.updateTarget();

    //Update the grid interface to make sure scored elements make it to the webpage
    Constants.gridInterface.update();
   }

  @Override
  public void disabledInit() {
    //Make sure all subsystems are disabled
    SubsystemManagerFactory.getInstance().disableAllSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    //Start all the subsystems in autonomous mode
    SubsystemManagerFactory.getInstance().notifyAutonomousStart();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    SubsystemManagerFactory.getInstance().notifyTeleopStart();

    //Send the grid interface into indicate so that I can update things quickly from autonomous
    Constants.gridInterface.indicateMode();

    //Send the grid automatically back to override mode after a few seconds of teleop
    new WaitCommand(6).andThen(new InstantCommand(Constants.gridInterface::overrideMode));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    //robotContainer.turret().setTarget(Math.toRadians(-90));
    robotContainer.requestTransition(RobotContainer.State.TESTING);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
