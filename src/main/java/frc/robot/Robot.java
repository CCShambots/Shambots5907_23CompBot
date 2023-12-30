package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.commands.WhileDisabledInstantCommand;
import frc.robot.subsystems.Lights.LightState;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private RobotContainer robotContainer;
  private final EventLoop checkModulesLoop = new EventLoop();

  @Override
  public void robotInit() {

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // TODO: Deal with this
        new PowerDistribution(1, ModuleType.kRev);
        break;
      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    robotContainer = new RobotContainer(checkModulesLoop);

    SubsystemManagerFactory.getInstance().registerSubsystem(robotContainer, false);
    SubsystemManagerFactory.getInstance().disableAllSubsystems();

    // TODO: What happened to the pathplanner server
    // if(!Constants.AT_COMP) {
    // PathPlannerLogging.startServer(5811);
    // }

    // Check the alliance from FMS when the bot turns on
    Constants.pullAllianceFromFMS(robotContainer);

    // TODO: Figure out how to add another periodic thing
    // Update the event loop for misaligned modules once every 10 seconds
    // addPeriodic(checkModulesLoop::poll, 10);

    new WaitCommand(2).andThen(robotContainer.syncAlliance()).schedule();

    new WaitCommand(2)
        .andThen(new WhileDisabledInstantCommand(() -> robotContainer.arm().pollAbsoluteAngles()))
        .schedule();

    // addPeriodic(() -> {if(!robotContainer.arm().isTransitioning())
    // robotContainer.arm().pullAbsoluteAngles();}, 2);

    // Logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update the grid interface to make sure scored elements make it to the webpage
    // Constants.gridInterface.update();
  }

  @Override
  public void disabledInit() {
    // Make sure all subsystems are disabled
    SubsystemManagerFactory.getInstance().disableAllSubsystems();

    robotContainer.lights().enable();

    robotContainer.arm().setShoulderFollower().schedule();
  }

  @Override
  public void disabledPeriodic() {
    if (!Constants.HAS_BEEN_ENABLED
        && robotContainer.turret().getMinimumAbsoluteErrorToStartingPos() > 3) {
      robotContainer.setFlag(RobotContainer.State.TURRET_STARTUP_MISALIGNMENT);
      robotContainer.lights().requestTransition(LightState.SOFT_STOP);
    } else {
      robotContainer.clearFlag(RobotContainer.State.TURRET_STARTUP_MISALIGNMENT);
      robotContainer.lights().requestTransition(LightState.DISABLED);
    }
  }

  @Override
  public void autonomousInit() {
    // Start all the subsystems in autonomous mode
    Constants.HAS_BEEN_ENABLED = true;
    SubsystemManagerFactory.getInstance().notifyAutonomousStart();

    robotContainer.arm().setShoulderFollower().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Constants.HAS_BEEN_ENABLED = true;

    SubsystemManagerFactory.getInstance().notifyTeleopStart();

    // robotContainer.scheduleEndgameBuzz();

    // Send the grid automatically back to override mode after a few seconds of teleop
    // new WaitCommand(6).andThen(new InstantCommand(Constants.gridInterface::overrideMode));

    robotContainer.arm().setShoulderFollower().schedule();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    Constants.HAS_BEEN_ENABLED = true;
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // robotContainer.turret().setTarget(robotContainer.getAutonomousCommand().getStartAngle());
    robotContainer.dt().setAllModules(Constants.SwerveDrivetrain.STOPPED_STATE);
    // robotContainer.requestTransition(RobotContainer.State.TESTING);

    robotContainer.arm().setShoulderFollower().schedule();
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
