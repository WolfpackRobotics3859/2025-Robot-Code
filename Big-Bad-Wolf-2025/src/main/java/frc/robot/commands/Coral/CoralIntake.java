// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.Global;
import frc.robot.subsystems.CoralPlacer;

/**
 * DEFAULT COMMAND
 * You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class CoralIntake extends Command 
{
  private final CoralPlacer m_CoralPlacer;

  /** 
   * Creates a new CoralIntake. 
   * 
   * @param p_CoralPlacer subsystem created in robotcontainer
   */
  public CoralIntake(CoralPlacer p_CoralPlacer) 
  {
    this.m_CoralPlacer = p_CoralPlacer;
    addRequirements(this.m_CoralPlacer);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() 
  {
    m_CoralPlacer.CoralPlacerRequest(
      CoralPlacerConstants.MOTOR.PLACER_WRIST, 
      Global.MODE.POSITION, 
      CoralPlacerConstants.CORAL_PLACER_WRIST_INTAKE_POSITION);// Set wrist to coral intaking position
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute()
  {
    m_CoralPlacer.IntakeCoral();// Stop Coral when it reaches correct point
  }

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
    // Intentionally empty
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
