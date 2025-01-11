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
import frc.robot.PhotonVision.AprilTagInfo.Area;
import frc.robot.PhotonVision.Camera.CAMERA_PLACEMENT;

public class CameraUtilities 
{
  // Camera Objects
  private static Camera m_FrontCamera = new Camera("FrontCamera", CAMERA_PLACEMENT.FRONT);
  private static Camera m_BackCamera = new Camera("BackCamera", CAMERA_PLACEMENT.BACK);
  private static Camera m_LeftCamera = new Camera("LeftCamera", CAMERA_PLACEMENT.LEFT);
  private static Camera m_RightCamera = new Camera("RightCamera", CAMERA_PLACEMENT.RIGHT);

  public static Camera[] m_Cameras = {m_FrontCamera, m_BackCamera, m_LeftCamera, m_RightCamera};

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
  public static class ComputerVisionTask 
  {

    public final static void alignWithCoralReef(AprilTagInfo info) 
    {
      //empty for now
    }

    public final static void alignWithAlgaeProcessor(AprilTagInfo info) 
    {
      //empty for now
    }

    public final static void alignWithCoralStation(AprilTagInfo info) 
    {
      //empty for now
    }

    public static void alignWithCage(AprilTagInfo info) 
    {
      //empty for now
    }
  }

  
  /** Uses a camera to update the odometry and help
   * the robot know where it is on the field
   * 
   * @param camera The camera to estimate the pose of the robot
   * @param m_odometry The odometry object to send sensor values to
   */
  public final static void updateVisionWithCamera(Camera camera, SwerveDrivePoseEstimator m_odometry) 
  {
    
    // Returns if the camera is not connected
    if (!camera.getPhotonCamera().isConnected()) 
    {
      return;
    }

    optionalPose = camera.getCameraPoseEstimator().update(null); // Updates the pose estimation of the camera
    int exceptionCount = camera.getCameraExceptionCount(); // Gets the current amount of exceptions the camera has encountered

    // try/catch statement to handle any errors from the camera
    try 
    {

      // Returns if the camera has not seen any april tags since last time this method was called
      if (!optionalPose.isPresent()) 
      {
        return;
      }

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


  /**Tells the robot to do a task if a specific camera senses an april tag in that task's region of the field
   * 
   * @param cam The camera to sense april tags from
   * @param taskArea The area of that task
   * @return True if an april tag has been sensed in that task's region of the field, false if not
   */
  public final static boolean doAprilTagTask(Camera cam, Area taskArea) 
  {

    // Checks if any of photon's pipelines has sensed anything
    List<PhotonPipelineResult> results = cam.getPhotonCamera().getAllUnreadResults();
    if (results.isEmpty()) return false; // Returns false if there were no unread results

    PhotonPipelineResult result = results.get(results.size() - 1); // Gets the last result
    AprilTagInfo info = new AprilTagInfo(-1); // Initializes an AprilTagInfo object with a negative ID for later configuration

    // Checks if the result(s) is an april tag
    if (!result.hasTargets()) 
    {
      return false;
    }
    
    // Iterates through all the april tags in the camera visions
    for (PhotonTrackedTarget target : result.getTargets()) 
    {
      info.setID(target.getFiducialId());
      info.setTarget(target);

      // if the april tag is not in the region of the task, we continue iterating thru the targets
      if (info.getArea() != taskArea) 
      {
        continue;
      }

      switch (taskArea) 
      {
        case ALGAE_PROCESSOR:
          ComputerVisionTask.alignWithAlgaeProcessor(info);
        case CLIMB:
          ComputerVisionTask.alignWithCage(info);
        case CORAL_REEF:
          ComputerVisionTask.alignWithCoralReef(info);
        case CORAL_STATION:
          ComputerVisionTask.alignWithCoralStation(info);
        case NONE:
          break;
        default:
          break;
      }
    }

    return true;
  }
}
