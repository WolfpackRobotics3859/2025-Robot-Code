// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;


  public Robot() throws FileVersionException, IOException, ParseException 
  {
    m_robotContainer = new RobotContainer();
    // Set the logger to log to the first flashdrive plugged in
    SignalLogger.setPath("/media/sda1/");
    // Explicitly start the logger
    SignalLogger.start();

    // Explicitly stop logging
    // If the user does not call stop(), then it's possible to lose the last few seconds of data
    SignalLogger.stop();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() 
  {
    // Explicitly stop logging
    // If the user does not call stop(), then it's possible to lose the last few seconds of data
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() 
  {
    // Explicitly start the logger
    SignalLogger.start();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
