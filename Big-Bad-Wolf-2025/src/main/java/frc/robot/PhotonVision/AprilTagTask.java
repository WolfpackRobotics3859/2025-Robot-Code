// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PhotonVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
  //TODO: code in these alignment/positioning methods
  /** 
   * A class containing all the alignment/positioning methods
   * using an AprilTagInfo object.
   */
  public class AprilTagTask 
  {
    private AprilTagInfo info;
    private CommandSwerveDrivetrain m_drivetrain;

    public AprilTagTask(AprilTagInfo initInfo, CommandSwerveDrivetrain initDrivetrain) 
    {
      setAprilTagInfo(initInfo);
      setCommandSwerveDrivetrain(initDrivetrain);
    }

    public final void setAprilTagInfo(AprilTagInfo newInfo) 
    {
      this.info = newInfo;
    }

    public final void setCommandSwerveDrivetrain(CommandSwerveDrivetrain newDrivetrain) 
    {
      this.m_drivetrain = newDrivetrain;
    }

    public final AprilTagInfo getAprilTagInfo() 
    {
      return this.info;
    }

    public final CommandSwerveDrivetrain getCommandSwerveDrivetrain() 
    {
      return this.m_drivetrain;
    }

    public final void alignWithCoralReef() 
    {
      // empty for now
    }

    public final void alignWithAlgaeProcessor() 
    {
      // empty for now
    }

    public final void alignWithCoralStation() 
    {
      // empty for now
    }

    public final void alignWithCage() 
    {
      // empty for now
    }

    public final void rotateToAprilTag() 
    {
      // Get the yaw of the AprilTag (degrees from the center of camera)
      double tagYawDegrees = info.getTarget().getYaw();

      // Use Pigeon2 sensor to get the robot's current yaw
      // let Pigeon2's yaw in degrees = yawReading
      // (yawReading % 360) = current yaw in degrees, from 0 to 360
      double robotYawDegrees = Math.floorMod((int)m_drivetrain.getPigeon2().getRotation2d().getDegrees(), 360);

      // Where the robot should face depending on the AprilTag's yaw
      double desiredYawDegrees = robotYawDegrees + tagYawDegrees;

      // Convert the yaw to radians (since Rotation2d's constructor uses radians)
      double desiredYawRadians = Units.degreesToRadians(desiredYawDegrees);

      // Rotate the robot using a command
      RotateToAngle rotationCommand = new RotateToAngle(m_drivetrain, new Rotation2d(desiredYawRadians));
      rotationCommand.schedule();
    }
  }
