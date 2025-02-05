// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command 
{
    /** Creates a new ClimbCommand. */
    Climb m_Climb;
    private double m_Voltage;
    public ClimbCommand(Climb p_Climb, double p_Voltage) 
    {
      this.m_Climb = p_Climb;
      m_Voltage = p_Voltage;
      addRequirements(this.m_Climb);
      // Use addRequirements() here to declare subsystem dependencies.
    }
      
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Climb.setClimbWristVoltage(this.m_Voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // m_Climb.setClimbWristVoltage(ClimbConstants.CLIMB_VOLTAGE_ZERO);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
