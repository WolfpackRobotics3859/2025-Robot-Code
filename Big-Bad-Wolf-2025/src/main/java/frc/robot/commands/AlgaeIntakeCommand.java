// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

// Creates a new Algae Intake command.
public class AlgaeIntakeCommand extends Command 
{
  private final AlgaeIntakeSubsystem m_AlgaeIntake;

  /**
   * Algae Intake command constructor.
   * @param algaeIntake Algae Intake subsystem.
   */
  public AlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntake) 
  {
    this.m_AlgaeIntake = algaeIntake;
    addRequirements(m_AlgaeIntake);
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
    m_AlgaeIntake.setRollerVoltage(AlgaeIntakeConstants.ALGAE_INTAKE_ROLLER_VOLTAGE);
    m_AlgaeIntake.setWristPosition(AlgaeIntakeConstants.ALGAE_INTAKE_WRIST_DOWN_POSITION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_AlgaeIntake.setRollerVoltage(AlgaeIntakeConstants.ALGAE_INTAKE_ROLLER_DEFAULT_VOLTAGE);
    m_AlgaeIntake.setWristPosition(AlgaeIntakeConstants.ALGAE_INTKAE_WRIST_DEFAULT_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
