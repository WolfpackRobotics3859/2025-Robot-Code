// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() 
  {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() 
  {
    //empty for now
  }

  @Override
  public void disabledPeriodic() 
  {
    //empty for now
  }

  @Override
  public void disabledExit() 
  {
    //empty for now
  }

  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {
    //empty for now
  }

  @Override
  public void autonomousExit() 
  {
    //empty for now
  }

  @Override
  public void teleopInit() 
  {
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() 
  {
    //empty for now
  }

  @Override
  public void teleopExit() 
  {
    //empty for now
  }

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() 
  {
    // empty for now
  }

  @Override
  public void testExit() 
  {
    //empty for now
  }

  @Override
  public void simulationPeriodic() 
  {
    //empty for now
  }
}
