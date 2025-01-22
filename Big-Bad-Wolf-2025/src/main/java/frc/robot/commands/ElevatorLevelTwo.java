// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Global;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorLevelTwo extends Command 
{
   private final Elevator m_Elevator;
  /** 
   * Creates a new ElevatorLevelTwo. 
   */
  public ElevatorLevelTwo(Elevator p_Elevator) 
  {
    this.m_Elevator = p_Elevator;
    addRequirements(this.m_Elevator);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() 
  {
    m_Elevator.elevatorRequest(Global.MODE.POSITION, ElevatorConstants.ELEVATOR_LEVEL_TWO);// Sets elevator to Level 2
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
    m_Elevator.elevatorRequest(Global.MODE.VOLTAGE, 0);// Once command ends shuts off motor voltage
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
