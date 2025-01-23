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

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private PowerDistribution PowerLogger;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    RobotContainer m_robotContainer = new RobotContainer();

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value for easier identification

    if (isReal()) {

        // If the robot is real, log live data (including power distribution) to a USB stick

        Logger.addDataReceiver(new WPILOGWriter()); // Log data to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Log data to NetworkTables for live logging

        // TODO: Find out the CAN ID of the power distribution system
        PowerLogger = new PowerDistribution(1, ModuleType.kCTRE); // Enables power distribution logging

    } else {

        // If the robot is in a simulation, read replay logs from AdvantageScope 
        // and save new data to a separate "_sim" log file

        setUseTiming(false); // Max logging speed
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log

    }

    Logger.start(); // Start logging - no more data receivers, replay sources, or metadata values may be added
    
    /* Use this method to log data
    * Logger.recordOutput();
    */
  }


  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {

    // Close the PowerDistribution logger to avoid memory leaks
    if (PowerLogger != null) {
      PowerLogger.close();
    }
  }

  @Override
  public void disabledPeriodic()
  {
    
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic()
  {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic()
  {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic()
  {

  }

  @Override
  public void simulationInit()
  {

  }

  @Override
  public void simulationPeriodic()
  {

  }
}
