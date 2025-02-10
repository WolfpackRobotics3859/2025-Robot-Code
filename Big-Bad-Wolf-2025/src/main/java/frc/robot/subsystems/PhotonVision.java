// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhotonConstants;
import frc.robot.subsystems.photonUtilities.AprilTagInfo;
import frc.robot.subsystems.photonUtilities.AprilTagTask;
import frc.robot.subsystems.photonUtilities.Camera;
import frc.robot.subsystems.photonUtilities.AprilTagInfo.Area;
import frc.robot.subsystems.photonUtilities.Camera.CAMERA_PLACEMENT;

public class PhotonVision extends SubsystemBase
{

  private static Optional<EstimatedRobotPose> optionalPose;
  private final CommandSwerveDrivetrain m_Drivetrain;
  public final Camera m_FrontCamera;
  public final Camera m_BackCamera;
  public final Camera m_RightCamera;
  public final Camera m_LeftCamera;

  public PhotonVision(CommandSwerveDrivetrain p_Drivetrain)
  {
    m_FrontCamera = PhotonConstants.frontCamera;
    m_BackCamera = PhotonConstants.backCamera;
    m_LeftCamera = PhotonConstants.leftCamera;    
    m_RightCamera = PhotonConstants.rightCamera;

    this.m_Drivetrain = p_Drivetrain;
  }

  public Optional<EstimatedRobotPose> positionEstimation(Camera camera)
  {
    // Creates an estimated position off most recent pipeline using camera pose estimator
    return camera.getCameraPoseEstimator().update(camera.getMostRecentPipeline());
  }
  
  /** Uses a camera to update the odometry and help
   * the robot know where it is on the field
   * 
   * @param camera The camera to estimate the pose of the robot
   * @param m_odometry The odometry object to send sensor values to
   */
  public final void updatePositionWithCamera(Camera camera, SwerveDrivePoseEstimator m_odometry) 
  {
    // Returns if the camera is not connected
    if (!camera.getPhotonCamera().isConnected()) return;

    // try/catch statement to handle any errors from the camera
    try 
    {    
      // calculates robot position using camera
      optionalPose = positionEstimation(camera);

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
      camera.setCameraExceptionCount(camera.getCameraExceptionCount() + 1);
    }
  }


  /** Uses the given Camera to see if we are near an AprilTag in the given area
   * 
   * @param camera We look for AprilTags using this camera
   * @param taskArea The area of the field
   * @param cameraResults 
   * @param m_drivetrain The drivetrain to 
   * @return True if an AprilTag has been sensed in that region of the field, False if not
   */
  public final AprilTagInfo aprilTagDetection(Camera camera, Area taskArea) 
  {
    AprilTagInfo info = new AprilTagInfo(-1); // new instance of AprilTagInfo

    PhotonPipelineResult cameraReading = camera.getMostRecentPipeline();

    // Checks if the result(s) is an AprilTag
    if (!cameraReading.hasTargets()) return info;
    System.out.println("AprilTag detected");

    // Iterates through all the AprilTags in the camera pipeline
    for (PhotonTrackedTarget target : cameraReading.getTargets()) 
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
  public final boolean doAprilTagTask(AprilTagInfo info, CommandSwerveDrivetrain m_drivetrain) 
  {
    // chekcs if the apriltag is valid then creates a new task
    if (!info.isValid()) return false;
    AprilTagTask task = new AprilTagTask(info, m_drivetrain);

    switch (info.getArea()) 
    {
      case ALGAE_PROCESSOR:
        task.alignWithAlgaeProcessor();
        break;
      case CLIMB:
        task.alignWithCage();
        break;
      case CORAL_REEF:
        System.out.println("CORAL REEF");
        task.alignWithCoralReef();
        break;
      case CORAL_STATION:
        task.alignWithCoralStation();
        break;
      case NONE:
        System.out.println("NO TASK");
        break;
      default:
        break;
    }
    return true;
  }

  
  @Override
  public void periodic()
    {

    }
}
