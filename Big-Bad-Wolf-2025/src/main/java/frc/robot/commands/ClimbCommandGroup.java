// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FunnelOpenCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.Climb;

// Creates a new Climb Command group.
public class ClimbCommandGroup extends SequentialCommandGroup 
{
  private final Climb m_Climb;

  /**
   * Climb command group constructor.
   * @param climb Climb subsystem.
   */
  public ClimbCommandGroup(Climb climb)
  {
    this.m_Climb = climb; 
    // Schedules Funnel Open command then Climb command sequentially.
    addCommands(
      new FunnelOpenCommand(m_Climb),
      new ClimbCommand(m_Climb)
    );
  }
}