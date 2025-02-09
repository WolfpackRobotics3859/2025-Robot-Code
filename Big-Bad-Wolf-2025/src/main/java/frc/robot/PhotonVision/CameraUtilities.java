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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PhotonVision.AprilTagInfo.Area;
import frc.robot.PhotonVision.Camera.CAMERA_PLACEMENT;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CameraUtilities extends SubsystemBase
{

  private static Optional<EstimatedRobotPose> optionalPose;
  public final Camera frontCamera;
  public final Camera backCamera;
  public final Camera rightCamera;
  public final Camera leftCamera;

  public CameraUtilities() 
  {
    frontCamera = new Camera("frontCamera", CAMERA_PLACEMENT.FRONT);
    backCamera = new Camera("backCamera", CAMERA_PLACEMENT.BACK);
    rightCamera = new Camera("rightCamera", CAMERA_PLACEMENT.RIGHT);
    leftCamera = new Camera("leftCamera", CAMERA_PLACEMENT.LEFT);
  }

  public Optional<EstimatedRobotPose> positionEstimation(Camera camera)
  {
    // Creates an estimated position based on 
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
        task.alignWithCoralReef();
        break;
      case CORAL_STATION:
        task.alignWithCoralStation();
        break;
      case NONE:
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
