// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeCleaner.AlgaeCleanerStowCommand;
import frc.robot.commands.Elevator.ElevatorGoHome;
import frc.robot.subsystems.AlgaeCleaner;
import frc.robot.subsystems.Elevator;

// Creates a new Algae Cleaner Intake to Stow Command group.
public class AlgaeCleanerIntakeToStow extends SequentialCommandGroup 
{
  private final Elevator m_Elevator;
  private final AlgaeCleaner m_AlgaeCleaner;

  /**
   * Algae Cleaner Intake to Stow command group constructor.
   * @param elevator Elevator subsystem.
   * @param algaeCleaner AlgaeCleaner subsystem.
   */
  public AlgaeCleanerIntakeToStow(Elevator elevator, AlgaeCleaner algaeCleaner) 
  {
    this.m_Elevator = elevator;
    this.m_AlgaeCleaner = algaeCleaner;
    // Schedules Algae Cleaner Stow command then Elevator Go Home command sequentially.
    addCommands(
      new AlgaeCleanerStowCommand(m_AlgaeCleaner),
      new ElevatorGoHome(m_Elevator)
    );
  }
}
