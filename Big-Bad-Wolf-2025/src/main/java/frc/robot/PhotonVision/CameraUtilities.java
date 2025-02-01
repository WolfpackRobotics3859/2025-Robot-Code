// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PhotonVision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.PhotonVision.AprilTagInfo.Area;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CameraUtilities 
{

  private static Optional<EstimatedRobotPose> optionalPose;

  public CameraUtilities() 
  {
    //empty for now
  }

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
      double robotYawDegrees = Math.floorMod((int)m_drivetrain.getPigeon2().getRotation2d().getDegrees(),360);

      // Where the robot should face depending on the AprilTag's yaw
      double desiredYawDegrees = robotYawDegrees + tagYawDegrees;

      // Convert the yaw to radians (since Rotation2d's constructor uses radians)
      double desiredYawRadians = Units.degreesToRadians(desiredYawDegrees);

      // Rotate the robot using a command
      RotateToAngle rotationCommand = new RotateToAngle(m_drivetrain, new Rotation2d(desiredYawRadians));
      rotationCommand.schedule();
    }
  }

  
  /** Uses a camera to update the odometry and help
   * the robot know where it is on the field
   * 
   * @param camera The camera to estimate the pose of the robot
   * @param m_odometry The odometry object to send sensor values to
   */
  public final void updateVisionWithCamera(Camera camera, SwerveDrivePoseEstimator m_odometry) 
  {
    
    // Returns if the camera is not connected
    if (!camera.getPhotonCamera().isConnected()) 
    {
      return;
    }

    optionalPose = camera.getCameraPoseEstimator().update(null); // Updates the pose estimation of the camera
    int exceptionCount = camera.getCameraExceptionCount();

    // try/catch statement to handle any errors from the camera
    try 
    {
      // Returns if the camera has not seen any AprilTags since last time this method was called
      if (!optionalPose.isPresent()) return;

      // Adds the camera sensor's input to the odometry
      EstimatedRobotPose pose = optionalPose.get();
      m_odometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    } 
    catch (Exception e) 
    {
      // Prints out the error of the camera and increments the camera's exception count
      System.out.println("ERROR! \tSource: " + camera.getCameraName() + "\tException: " + e);
      camera.setCameraExceptionCount(exceptionCount + 1);
    }
  }


  /** Uses the given Camera to see if we are near an AprilTag in the given area
   * 
   * @param cam We look for AprilTags using this camera
   * @param taskArea The area of the field
   * @param m_drivetrain The drivetrain to 
   * @return True if an AprilTag has been sensed in that region of the field, False if not
   */
  public final AprilTagInfo cameraSensesAprilTag(Camera cam, Area taskArea) 
  {
    AprilTagInfo info = new AprilTagInfo(-1); // AprilTagInfo object for later configuration

    // Checks if any of photon's pipelines has sensed anything
    List<PhotonPipelineResult> results = cam.getPhotonCamera().getAllUnreadResults();

    // Returns false if there were no unread results
    if (results.isEmpty()) 
    {
      return info;
    }

    PhotonPipelineResult result = results.get(results.size() - 1); // Gets the last result

    // Checks if the result(s) is an AprilTag
    if (!result.hasTargets())
    {
      return info;
    }
    
    System.out.println("AprilTag detected");

    // Iterates through all the AprilTags in the camera visions
    for (PhotonTrackedTarget target : result.getTargets()) 
    {
      info.setID(target.getFiducialId());
      info.setTarget(target);

      if (info.getArea() == Area.NONE || info.getArea() == taskArea) 
      {
        break;
      }
    }

    return info;
  }

/** Uses the given AprilTagInfo object to do a task (like alignment) in the given Area
 * 
 * @param info The AprilTagInfo object which contains information about rotation and ID
 * @param taskArea The region of the field to do the task in
 * @param m_drivetrain The drivetrain to use when aligning/positioning
 * @return
 */
public final boolean doAprilTagTask(AprilTagInfo info, Area taskArea, CommandSwerveDrivetrain m_drivetrain) 
{
    if (!info.isValid()) return false;

    AprilTagTask task = new AprilTagTask(info, m_drivetrain);

    switch (taskArea) 
    {
      case ALGAE_PROCESSOR:
        task.alignWithAlgaeProcessor();
      case CLIMB:
        task.alignWithCage();
      case CORAL_REEF:
        task.alignWithCoralReef();
      case CORAL_STATION:
        task.alignWithCoralStation();
      case NONE:
      default:
        break;
    }

    return true;

  }
}
