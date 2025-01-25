// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.Climb;

// Creates a new Climb command.
public class ClimbCommand extends Command 
{
  private final Climb m_Climb;

  /**
   * Climb command constructor.
   * @param climb Climb subsystem.
   */
  public ClimbCommand(Climb climb) 
  {
    this.m_Climb = climb;
    addRequirements(m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Climb.setClimbWristPosition(ClimbConstants.CLIMB_WRIST_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Intentionally Empty.
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
