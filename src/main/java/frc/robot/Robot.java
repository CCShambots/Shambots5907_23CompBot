package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.subsystems.Lights.LightState;


public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private final EventLoop checkModulesLoop = new EventLoop();

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer(checkModulesLoop);

    SubsystemManagerFactory.getInstance().registerSubsystem(robotContainer, false);
    SubsystemManagerFactory.getInstance().disableAllSubsystems();

    // LiveWindow.disable AllTelemetry();

    if(!Constants.AT_COMP) {
      PathPlannerServer.startServer(5811);
    }

    //Run the module control loops every 5 ms
//    addPeriodic(robotContainer.runArmControlLoops(), 0.005);

    //Check the alliance from FMS when the bot turns on
    Constants.pullAllianceFromFMS(robotContainer);

    //Update the event loop for misaligned modules once every 10 seconds
    addPeriodic(checkModulesLoop::poll, 10);

    new WaitCommand(2).andThen(robotContainer.syncAlliance()).schedule();

    new WaitCommand(2).andThen(new WhileDisabledInstantCommand(() -> robotContainer.arm().pullAbsoluteAngles())).schedule();

    //Logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Update the grid interface to make sure scored elements make it to the webpage
    Constants.gridInterface.update();
   }

  @Override
  public void disabledInit() {
    //Make sure all subsystems are disabled
    SubsystemManagerFactory.getInstance().disableAllSubsystems();

    robotContainer.lights().enable();
  }

  @Override
  public void disabledPeriodic() {
    if (!Constants.HAS_BEEN_ENABLED && robotContainer.turret().getMinimumAbsoluteErrorToStartingPos() > 3) {
      robotContainer.setFlag(RobotContainer.State.TURRET_STARTUP_MISALIGNMENT);
      robotContainer.lights().requestTransition(LightState.SOFT_STOP);
    }
    else {
      robotContainer.clearFlag(RobotContainer.State.TURRET_STARTUP_MISALIGNMENT);
      robotContainer.lights().requestTransition(LightState.DISABLED);
    }
  }

  @Override
  public void autonomousInit() {
    //Start all the subsystems in autonomous mode
    Constants.HAS_BEEN_ENABLED = true;
    SubsystemManagerFactory.getInstance().notifyAutonomousStart();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    Constants.HAS_BEEN_ENABLED = true;

    SubsystemManagerFactory.getInstance().notifyTeleopStart();


    robotContainer.scheduleEndgameBuzz();
    //Send the grid interface into indicate so that I can update things quickly from autonomous
    Constants.gridInterface.indicateMode();

    //Send the grid automatically back to override mode after a few seconds of teleop
    new WaitCommand(6).andThen(new InstantCommand(Constants.gridInterface::overrideMode));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    Constants.HAS_BEEN_ENABLED = true;
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    robotContainer.turret().setTarget(robotContainer.getAutonomousCommand().getStartAngle());
    robotContainer.dt().setAllModules(Constants.SwerveDrivetrain.STOPPED_STATE);
    // robotContainer.requestTransition(RobotContainer.State.TESTING);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    Constants.HAS_BEEN_ENABLED = true;
  }

  @Override
  public void simulationPeriodic() {}
}
