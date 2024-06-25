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
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.Limelight;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  /**
   * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm,
   * very little robot logic should actually be handled in the Robot periodic methods (other than the scheduler calls).
   * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
   */
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  @SuppressWarnings("unused") // Suppress dead code warnings since Java thinks BuildConstants values will be static.
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "Crescendo2024"); // Set a metadata value

    // NOTE: If there's an error with "BuildConstants cannot be resolved", build the program.
    // The BuildConstants file is generated on build and not checked into version control since it can be invalid, cause merge conflicts, etc.
    Logger.recordMetadata("GitInformation", 
      "Hash " +
      BuildConstants.GIT_SHA +
      " on " + 
      BuildConstants.GIT_BRANCH +
      (BuildConstants.DIRTY == 1 ? "*" : "")
    );
    Logger.recordMetadata("BuiltOn", BuildConstants.BUILD_DATE); // Set a metadata value
    if(DriverStation.isFMSAttached()) {
      // Note: This won't record metadata if the robot is turned on before we're connected to the FMS.
      Logger.recordMetadata("EventName", DriverStation.getEventName());
      Logger.recordMetadata("GameSpecificMessage", DriverStation.getGameSpecificMessage());
      Logger.recordMetadata("MatchNumber", Integer.toString(DriverStation.getMatchNumber()));
      Logger.recordMetadata("MatchType", DriverStation.getMatchType().toString());
      Logger.recordMetadata("ReplayNumber", Integer.toString(DriverStation.getReplayNumber()));
      Logger.recordMetadata("Alliance", DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).toString());
      Logger.recordMetadata("AllianceLocation", Integer.toString(DriverStation.getLocation().orElse(-1)));
    }

    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        // setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.registerURCL(URCL.startExternal());
    
    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    // Instantiate our RobotContainer.  This will perform all our button bindings, put our
    // autonomous chooser on the dashboard, and do anything else required for initialization.
    robotContainer = new RobotContainer();
    
    Limelight.getInstance().initiaize();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
    double startTime = Logger.getRealTimestamp();
    CommandScheduler.getInstance().run();
    Logger.recordOutput("LoggedRobot/CommandSchedulerTime", (Logger.getRealTimestamp() - startTime) / 1000);

    setNetworkTablesFlushEnabled(!DriverStation.isFMSAttached());

    Logger.recordOutput("JVMStats/FreeMemory", Runtime.getRuntime().freeMemory());
    Logger.recordOutput("JVMStats/TotalMemory", Runtime.getRuntime().totalMemory());
    Logger.recordOutput("JVMStats/UsedMemory", Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopInit() {
    robotContainer.teleopEnable();
    Superstructure.getInstance().resetSubsystemsForTeleop();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {}
}
