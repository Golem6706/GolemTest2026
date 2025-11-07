// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.RobotMode;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

  public enum RobotName{
    // Team 5516 dev bot
    TEAM_5516_DEVBOT_HYDROXIDE_I,
    // Team 5516 comp bot
    TEAM_5516_COMPBOT_HYDROXIDE_II,
    // Team 6706 comp bot
    TEAM_6706_COMPBOT
  }

  private static final RobotMode JAVA_SIM_MODE = RobotMode.SIM;
  public static final RobotMode CURRENT_ROBOT_MODE = isReal() ? RobotMode.REAL : JAVA_SIM_MODE;
  public static final RobotName CURRENT_ROBOT = RobotName.TEAM_6706_COMPBOT;

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Record metadata
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

    switch(CURRENT_ROBOT_MODE){
      case REAL -> {
        //Running on a real robot, log to USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Publsh data to NetworkTables
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        // Running a physics simulator
        // Log to CodeDirectory/logs if want to test logging system in a simulation mode
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        // Replaying from a log, set up replay source
        setUseTiming(false);  // Run as fast as possible
        String logPathString = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPathString));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPathString, "_replayed"),
              WPILOGWriter.AdvantageScopeOpenBehavior.ALWAYS));
      }
    }

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Start AdvantageKit logger
    Logger.start();

    // Optimization, Set the mian Thread hightest priority
    Thread.currentThread().setPriority(Thread.MAX_PRIORITY);

    SignalLogger.enableAutoLogging(false);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  private Command testCommand = Commands.none();

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(testCommand = robotContainer.getTestCommand());
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    testCommand.cancel();
    robotContainer.configureButtonBindings();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateFieldSimAndDisplay();
  }
}
