// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FunnelClosedCommand;
import frc.robot.subsystems.Climb;

// Creates a new Climb Stow command group.
public class ClimbStowCommandGroup extends SequentialCommandGroup 
{
  Climb m_Climb;

  /**
   * Creates a new Climb stow command group.
   * @param climb Climb subsystem.
   */
  public ClimbStowCommandGroup(Climb climb) 
  {
    m_Climb = climb;
    addCommands(
      m_Climb.goToStowPosition(),
      new FunnelClosedCommand(m_Climb)
    );
  }
}
