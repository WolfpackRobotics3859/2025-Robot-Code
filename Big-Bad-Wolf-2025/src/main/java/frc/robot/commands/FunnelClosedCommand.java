// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.subsystems.CoralPlacer;

// Creates a new Funnel Closed command.
public class FunnelClosedCommand extends Command 
{
  private final CoralPlacer m_CoralPlacer;

  /**
   * Funnel Closed command constructor.
   * @param coralPlacer Coral Placer subsystem.
   */
  public FunnelClosedCommand(CoralPlacer coralPlacer) 
  {
    this.m_CoralPlacer = coralPlacer;
    addRequirements(m_CoralPlacer);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    // Intentionally Empty.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_CoralPlacer.setFunnelWristPosition(CoralPlacerConstants.CORAL_FUNNEL_CLOSED_POSITION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Intentionally Empty.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
