// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.PackLog;

public class Robot extends TimedRobot 
{
  private PackLog m_PackLog;
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() 
  {
    DataLogManager.start();
    m_PackLog = new PackLog("Robot");
    
    m_robotContainer = new RobotContainer();
    // Set the logger to log to the first flashdrive plugged in
    SignalLogger.setPath("/media/sda1/");
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() 
  {
    this.m_PackLog.Log("Entering DISABLED mode.");
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void disabledExit() 
  {
    this.m_PackLog.Log("Exiting DISABLED mode.");
    SignalLogger.start();
  }

  @Override
  public void autonomousInit() 
  {
    this.m_PackLog.Log("Entering AUTONOMOUS mode.");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void autonomousExit()
  {
    this.m_PackLog.Log("Exiting AUTONOMOUS mode.");
  }

  @Override
  public void teleopInit() 
  {
    this.m_PackLog.Log("Entering TELEOP mode.");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.InitializeDefaultCommands();
  }

  @Override
  public void teleopPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void teleopExit() 
  {
    this.m_PackLog.Log("Exiting TELEOP mode.");
  }

  @Override
  public void testInit() 
  {
    this.m_PackLog.Log("Entering TEST mode.");
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void testExit() 
  {
    this.m_PackLog.Log("Exiting TEST mode.");
  }

  @Override
  public void simulationPeriodic() 
  {
    // Intentionally Empty
  }
}
