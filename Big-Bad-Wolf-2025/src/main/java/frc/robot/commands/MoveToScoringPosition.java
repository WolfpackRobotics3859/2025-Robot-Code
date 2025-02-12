// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/*FIX WHEN WE HAVE VISION */
/* 
public class MoveToScoringPosition extends Command
{
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonVision vision;
  private final boolean goLeft;
  private Pose2d targetPose;
  private int desiredTag;
  

  private static final double OFFSET = 0.12065; //distance between center of apriltag and center of left/right reef prong ; may need tuning

  public MoveToScoringPosition(CommandSwerveDrivetrain drivetrain, PhotonVision vision, boolean goLeft) 
  {
      this.drivetrain = drivetrain;
      this.vision = vision;
      this.goLeft = goLeft;
      addRequirements(drivetrain);
  }


  @Override
  public void initialize()
  {
    //intentionally empty
  }


  @Override
  public void execute() 
  {
    Pose2d tagPose = vision.getAprilTagPose(); // Get AprilTag position

      if (tagPose != null) 
      {
        if (isAligned != true)
        {
          vision.alignmentMethod(desiredTag); //should align the robot with the desired tag
        }
        else
        {
          tagPose = vision.getAprilTagPose(); // Get AprilTag position again, should be centered this time

          //if goLeft is true, moves to the left by the offset amount; if goLeft is false (going right), moves to the right by the offset amount
          double targetX = tagPose.getX() + (goLeft ? -OFFSET : OFFSET);     
          //creates the target pose     
          targetPose = new Pose2d(targetX, tagPose.getY(), tagPose.getRotation());

          //tells the drivetrain to move to the target pose
          drivetrain.driveTo(targetPose);
        }
      }
  }


  @Override
  public void end(boolean interrupted) 
  {
    // Intentionally Empty.
  }


  @Override
  public boolean isFinished() 
  {
      return targetPose != null && drivetrain.atTargetPosition(targetPose);
  }
}
*/