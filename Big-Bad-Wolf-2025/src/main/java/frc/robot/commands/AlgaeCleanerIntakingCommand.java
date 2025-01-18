// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeCleanerConstants;
import frc.robot.subsystems.AlgaeCleaner;

// Creates a new Algae Cleaner command.
public class AlgaeCleanerIntakingCommand extends Command 
{
  private final AlgaeCleaner m_AlgaeCleaner;

  /**
   * Algae Cleaner command constructor.
   * @param algaeCleaner Algae Cleaner subsystem.
   */
  public AlgaeCleanerIntakingCommand(AlgaeCleaner algaeCleaner) 
  {
    this.m_AlgaeCleaner = algaeCleaner;
    addRequirements(m_AlgaeCleaner);
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
    m_AlgaeCleaner.setCleanerWristPosition(AlgaeCleanerConstants.ALGAE_CLEANER_WRIST_INTAKING_POSITION);
    m_AlgaeCleaner.setCleanerRollerVoltage(AlgaeCleanerConstants.ALGAE_CLEANER_ROLLER_VOLTAGE);
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