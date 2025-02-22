// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LookAtAprilTag extends Command 
{
  /** Creates a new AlignWithAprilTag. */
  private final PhotonVision m_PhotonVision;
  public LookAtAprilTag(PhotonVision p_PhotonVision) {
    this.m_PhotonVision = p_PhotonVision;
    addRequirements(this.m_PhotonVision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_PhotonVision.aprilTagTaskReady())
    {
      m_PhotonVision.rotateToAprilTag(m_PhotonVision.getClosestTag());
    }
    System.out.println("NOT READY");
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
