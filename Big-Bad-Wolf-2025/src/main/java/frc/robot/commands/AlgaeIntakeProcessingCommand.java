// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;

// Creates a new Algae Intake Processing command.
public class AlgaeIntakeProcessingCommand extends Command 
{
  private final AlgaeIntake m_AlgaeIntake;
  
  /**
   * Algae Intake Processing command constructor.
   * @param algaeIntake Algae Intake subsystem.
   */
  public AlgaeIntakeProcessingCommand(AlgaeIntake algaeIntake) 
  {
    this.m_AlgaeIntake = algaeIntake;
    addRequirements(m_AlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_AlgaeIntake.setRollerVoltage(AlgaeIntakeConstants.ALGAE_INTAKE_ROLLER_PROCESSING_VOLTAGE);
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