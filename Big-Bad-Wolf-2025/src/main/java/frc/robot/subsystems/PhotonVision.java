// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhotonConstants;
import frc.robot.subsystems.photonUtilities.AprilTagInfo;
import frc.robot.subsystems.photonUtilities.Camera;
import frc.robot.subsystems.photonUtilities.AprilTagInfo.Area;
import frc.robot.subsystems.photonUtilities.Camera.CAMERA_PLACEMENT;

public class PhotonVision extends SubsystemBase
{
  private AprilTagInfo closestAprilTag;
  private double taskRange;
  public int desiredAprilTagId;
  private Optional<EstimatedRobotPose> optionalPose;
  private final CommandSwerveDrivetrain m_Drivetrain;
  public final Camera m_FrontCamera;
  // public final Camera m_BackCamera;
  // public static Camera m_RightCamera;
  // public final Camera m_LeftCamera;

  private List<Camera> enabledCameras;
  // private List<PhotonPipelineResult> enabledCamerasFeed;

  public PhotonVision(CommandSwerveDrivetrain p_Drivetrain)
  {
    enabledCameras = new ArrayList<>();
    m_FrontCamera = PhotonConstants.frontCamera;
    // m_BackCamera = PhotonConstants.backCamera;
    // m_LeftCamera = PhotonConstants.leftCamera;    
    // m_RightCamera = PhotonConstants.rightCamera;
    enabledCameras.add(m_FrontCamera);
    // enabledCameras.add(m_BackCamera);
    // enabledCameras.add(m_RightCamera);
    // enabledCameras.add(m_LeftCamera);

    this.m_Drivetrain = p_Drivetrain;

    taskRange = 1; // In meters supposedly
  }

  public void setDesiredId(int id)
  {
    desiredAprilTagId = id;
  }

  public Translation2d getTagTranslation2d(AprilTagInfo info)
  {
    return info.getTarget().getBestCameraToTarget().getTranslation().toTranslation2d();
  }

  public Optional<EstimatedRobotPose> positionEstimation(Camera camera)
  {
    // Creates an estimated position off most recent pipeline using camera pose estimator
    return camera.getCameraPoseEstimator().update(camera.getMostRecentPipeline());
  }

  public final Transform3d robotToTargetTransform3d(AprilTagInfo info)
    {
      return info.getTarget()
      .getBestCameraToTarget()
      .plus(info.getCameraWitness().getCamera3DPosition());
    }
  
  /** Uses a camera to update the odometry and help
   * the robot know where it is on the field
   * 
   * @param camera The camera to estimate the pose of the robot
   * @param m_odometry The odometry object to send sensor values to
   */
  public final void updatePositionWithCamera(Camera camera, SwerveDrivetrain m_odometry) 
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

  private AprilTagInfo getInRangeTag(Camera camera)
  {
    AprilTagInfo info = new AprilTagInfo(-1); // new instance of AprilTagInfo

    PhotonPipelineResult cameraReading = camera.getMostRecentPipeline();

    // Checks if the result(s) is an AprilTag
    if (!cameraReading.hasTargets()) return info;
    System.out.println("AprilTag detected");

    // Iterates through all the AprilTags in the camera pipeline
    PhotonTrackedTarget target = cameraReading.getTargets().get(cameraReading.getTargets().size()-1);
    double targetDistance = target.getBestCameraToTarget().plus(camera.getCamera3DPosition())
      .getTranslation()
        .toTranslation2d()
          .getNorm();

    if (targetDistance <= taskRange)
    {
      info.setID(target.getFiducialId());
      info.setTarget(target);
      info.setCameraWitness(camera);
      return info;
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
  public final boolean aprilTagTaskCompleted(CommandSwerveDrivetrain m_drivetrain) 
  {
    // chekcs if the apriltag is valid
    if (!closestAprilTag.isValid()) return false;
    
     if (desiredAprilTagId == closestAprilTag.getID())
     {
        align(closestAprilTag);
     }

    return true;
  }

  public final void align(AprilTagInfo aprilTag) 
  {
    PIDController lateralController = new PIDController(1, 0, 0);
    PIDController angularController = new PIDController(1, 0, 0);
    angularController.enableContinuousInput(-Math.PI, Math.PI);
    angularController.setTolerance(1);

    double lateralOffest = getTagTranslation2d(aprilTag).getX();
    double angularOffset = aprilTag.getTarget().getYaw();

    double lateralCorrection = lateralController.calculate(lateralOffest, 0);
    double angularCorrection = Units.degreesToRadians(angularController.calculate(angularOffset, 0));

    ChassisSpeeds speeds = new ChassisSpeeds(0, lateralCorrection, angularCorrection);

    m_Drivetrain.driveRobotRelative(speeds);
  }

  // public final void rotateToAprilTag(AprilTagInfo info) 
  // {
  //   // Get the yaw of the AprilTag (degrees from the center of camera)
  //   double tagYawOffset = info.getTarget().getYaw();

  //   // Use Pigeon2 sensor to get the robot's raw yaw in degrees
  //   double robotYawRaw = m_Drivetrain.getPigeon2().getRotation2d().getDegrees();
  //   // (yawReading % 360) = current yaw in degrees 
  //   double robotYawDegrees = ((robotYawRaw % 360) + 360) % 360; // Add 360 to find positive coterminal angle then % to get rid of values over 360

  //   // Where the robot should face depending on the AprilTag's yaw
  //   double desiredYawDegrees = robotYawDegrees + tagYawOffset;

  //   // Convert the yaw to radians (since Rotation2d's constructor uses radians)
  //   double desiredYawRadians = Units.degreesToRadians(desiredYawDegrees);

  //   Pose2d desiredPose = new Pose2d(0, 0, new Rotation2d(desiredYawRadians)); 
  // }

  
  @Override
  public void periodic()
  {
    for(Camera camera : enabledCameras)
    {
      camera.updateUnreadPipelines();
      camera.updateMostRecentPipeline();
      updatePositionWithCamera(camera, m_Drivetrain);
      closestAprilTag = getInRangeTag(camera);
      // enabledCamerasFeed.add(camera.getMostRecentPipeline());
    }
  }
}
