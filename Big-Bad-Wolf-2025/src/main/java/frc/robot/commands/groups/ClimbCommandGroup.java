// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FunnelOpenCommand;
import frc.robot.subsystems.Climb;

// Creates a new Climb command group.
public class ClimbCommandGroup extends SequentialCommandGroup 
{
  Climb m_Climb;

  /**
   * Constructor for Climb command group.
   * @param climb Climb subsystem.
   */
  public ClimbCommandGroup(Climb climb)
  {
    m_Climb = climb;
    addCommands(
      // Calls funnel to separate to a specified position within upper and lower bounds.
      new FunnelOpenCommand(m_Climb),
      // Calls Climb arm to go to a specified position.
      m_Climb.goToClimbPosition()
    );
  }
}
