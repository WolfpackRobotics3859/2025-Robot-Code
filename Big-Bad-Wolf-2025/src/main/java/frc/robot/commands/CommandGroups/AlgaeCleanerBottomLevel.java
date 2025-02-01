// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeCleaner.AlgaeCleanerIntakingCommand;
import frc.robot.commands.Elevator.ElevatorUp;
import frc.robot.subsystems.AlgaeCleaner;
import frc.robot.subsystems.Elevator;

// Creates a new Algae Cleaner Bottom level Command group.
public class AlgaeCleanerBottomLevel extends SequentialCommandGroup 
{
  private final Elevator m_Elevator;
  private final AlgaeCleaner m_AlgaeCleaner;

  /**
   * Algae Cleaner Bottom level command group constructor.
   * @param elevator Elevator subsystem.
   * @param algaeCleaner AlgaeCleaner subsystem.
   */
  public AlgaeCleanerBottomLevel(Elevator elevator, AlgaeCleaner algaeCleaner) 
  {
    this.m_Elevator = elevator;
    this.m_AlgaeCleaner = algaeCleaner;
    // Schedules Elevator Up command then Algae Cleaner Intaking command sequentially.
    addCommands(
      new ElevatorUp(m_Elevator),
      new AlgaeCleanerIntakingCommand(m_AlgaeCleaner)
    );
  }
}
