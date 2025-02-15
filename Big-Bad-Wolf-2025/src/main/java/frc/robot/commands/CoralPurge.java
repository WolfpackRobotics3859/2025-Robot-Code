// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.Global;
import frc.robot.constants.CoralPlacerConstants.MOTOR;
import frc.robot.constants.Global.MODE;
import frc.robot.subsystems.CoralPlacer;

/**
 * You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands 
 */
public class CoralPurge extends Command 
{
  private final CoralPlacer m_CoralPlacer;
  /**
   * Creates a new CoralOutake. 
   * 
   * @param p_CoralPlacer subsystem created in robotcontainer
   */
  public CoralPurge(CoralPlacer p_CoralPlacer) 
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
    m_CoralPlacer.CoralPlacerRequest(MOTOR.PLACER_WRIST, MODE.POSITION, CoralPlacerConstants.CORAL_PLACER_WRIST_PLACE_POSITION);
    m_CoralPlacer.CoralPlacerRequest(
      CoralPlacerConstants.MOTOR.PLACER_ROLLER, 
      Global.MODE.VOLTAGE, 
      CoralPlacerConstants.CORAL_PLACER_PURGE_VOLTAGE);// Applies positive voltage to purge coral
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() 
  {
    // Intentionally empty
  }

  /** 
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
    m_CoralPlacer.CoralPlacerRequest(CoralPlacerConstants.MOTOR.PLACER_ROLLER, Global.MODE.VOLTAGE, 0);// Stops voltage
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
