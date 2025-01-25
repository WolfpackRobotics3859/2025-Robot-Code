// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;

// Creates a new Algae Ground Intake command.
public class AlgaeIntakeGroundCommand extends Command 
{
  private final AlgaeIntake m_AlgaeIntake;

  public AlgaeIntakeGroundCommand(AlgaeIntake algaeIntake) 
  {
    this.m_AlgaeIntake = algaeIntake;
    addRequirements(m_AlgaeIntake);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_AlgaeIntake.setWristPosition(AlgaeIntakeConstants.ALGAE_INTAKE_WRIST_GROUND_POSITION);
    m_AlgaeIntake.setRollerVoltage(AlgaeIntakeConstants.ALGAE_INTAKE_ROLLER_VOLTAGE);
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
