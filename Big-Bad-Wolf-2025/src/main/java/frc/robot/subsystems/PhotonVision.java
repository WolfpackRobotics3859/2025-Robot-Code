// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhotonConstants;
import frc.robot.subsystems.photonUtilities.AprilTagInfo;
import frc.robot.subsystems.photonUtilities.Camera;
import frc.robot.utilities.SubsystemManager;
import frc.robot.utilities.subsystemManager.SubsystemAddedEvent;
import frc.robot.utilities.subsystemManager.SubsystemAddedListener;

public class PhotonVision extends SubsystemBase implements SubsystemAddedListener
{
  private AprilTagInfo closestAprilTag;
  private double taskRange;
  public int desiredAprilTagId;
  private Optional<EstimatedRobotPose> optionalPose;
  // public final Camera m_FrontCamera;
  // public final Camera m_BackCamera;
  public Camera m_RightCamera;
  // public final Camera m_LeftCamera;

  private List<Camera> enabledCameras;
  // private List<PhotonPipelineResult> enabledCamerasFeed;

  private SubsystemManager m_Subsystems;
  private CommandSwerveDrivetrain m_Drivetrain;

  private Field2d field;

  PIDController xyController;
  PIDController angularController;

  public PhotonVision(SubsystemManager manager)
  {
    this.m_Subsystems = manager;
    this.m_Drivetrain = this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
   
    enabledCameras = new ArrayList<>();
    // m_FrontCamera = PhotonConstants.frontCamera;
    // m_BackCamera = PhotonConstants.backCamera;
    // m_LeftCamera = PhotonConstants.leftCamera;    
    m_RightCamera = PhotonConstants.rightCamera;
    // enabledCameras.add(m_FrontCamera);
    // enabledCameras.add(m_BackCamera);
    enabledCameras.add(m_RightCamera);
    // enabledCameras.add(m_LeftCamera);

    taskRange = 1.3; // In meters supposedly

    xyController = new PIDController(1, 0, 0); 
    angularController = new PIDController(1, 0, 0); 
    angularController.enableContinuousInput(-Math.PI, Math.PI);
    angularController.setTolerance(0.5);
  }

  public void setDesiredId(int id)
  {
    desiredAprilTagId = id;
  }

  public AprilTagInfo getClosestTag()
  {
    return closestAprilTag;
  }

  public final Transform3d getTargetTransform3d(AprilTagInfo info)
    {
      return info.getCameraWitness()
      .getCamera3DPosition()
      .plus(
        info.getTarget()
        .getBestCameraToTarget());
    }


  public Optional<EstimatedRobotPose> positionEstimation(Camera camera)
  {
    // Creates an estimated position off most recent pipeline using camera pose estimator
    return camera
    .getCameraPoseEstimator()
    .update(
      camera.getMostRecentPipeline());
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
  

  private AprilTagInfo getInRangeTag(Camera camera)
  {
    AprilTagInfo info = new AprilTagInfo(-1); // new instance of AprilTagInfo

    PhotonPipelineResult cameraReading = camera.getMostRecentPipeline();

    // Checks if the result(s) is an AprilTag
    if (!cameraReading.hasTargets()) return info;
    System.out.println("AprilTag detected");

    // Iterates through all the AprilTags in the camera pipeline
    Pose2d robotPosition = m_Drivetrain.getPose2d();
    PhotonTrackedTarget target = cameraReading.getTargets().get(cameraReading.getTargets().size()-1);
    double targetDistance = target.getBestCameraToTarget().plus(camera.getCamera3DPosition())
      .getTranslation()
        .toTranslation2d()
          .getDistance(new Translation2d(robotPosition.getX(), robotPosition.getY()));

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
  public final boolean aprilTagTaskReady() 
  {
    // chekcs if the apriltag is valid
    if (!closestAprilTag.isValid()) return false;
    
     if (desiredAprilTagId == closestAprilTag.getID() && (desiredAprilTagId != -1 || closestAprilTag.getID() != -1))
     {
        return true;
     }
    return false;
  }

  public final void align(AprilTagInfo aprilTag) 
  {
    Pose2d currentRobotPosition2d = m_Drivetrain.getPose2d();
    Translation2d aprilTagTranslation2d = getTargetTransform3d(aprilTag).getTranslation().toTranslation2d();

    Rotation2d aprilTagFacingAwayAngle = aprilTag.getTarget().getBestCameraToTarget().getRotation().toRotation2d();
    Translation2d alignedTranslation2d = currentRobotPosition2d.getTranslation().rotateAround(aprilTagTranslation2d, aprilTagFacingAwayAngle);
    
    double alignedX = alignedTranslation2d.getX();
    double alignedY = alignedTranslation2d.getY();

    double xCorrection = xyController.calculate(currentRobotPosition2d.getX(), alignedX);
    double yCorrection = xyController.calculate(currentRobotPosition2d.getY(), alignedY);
    double angularCorrection = Units.degreesToRadians(angularController.calculate(m_Drivetrain.getPigeon2().getRotation2d().getRadians(), aprilTagFacingAwayAngle.getRadians()));
    
    ChassisSpeeds speeds = new ChassisSpeeds(xCorrection, yCorrection, angularCorrection);
    m_Drivetrain.driveRobotRelative(speeds);
  
  }

  public final void rotateToAprilTag(AprilTagInfo info) 
  {
    // Get the yaw of the AprilTag (degrees from the center of camera)
    double tagYawOffset = Units.degreesToRadians(info.getTarget().getYaw());

    // Use Pigeon2 sensor to get the robot's raw yaw in degrees
    double robotYawRaw = m_Drivetrain.getPigeon2().getRotation2d().getDegrees();
    // (yawReading % 360) = current yaw in degrees 
    double robotYaw = Units.degreesToRadians(((robotYawRaw % 360) + 360) % 360); // Add 360 to find positive coterminal angle then % to get rid of values over 360
    // Where the robot should face depending on the AprilTag's yaw

    try (PIDController angularController = new PIDController(1, 0, 0)) {
      angularController.enableContinuousInput(-Math.PI, Math.PI);
      angularController.setTolerance(0.5);

      double angularCorrection = angularController.calculate(robotYaw, tagYawOffset);

      ChassisSpeeds speeds = new ChassisSpeeds(0,0, angularCorrection);
      m_Drivetrain.driveRobotRelative(speeds);
    }
  }


  @Override
  public void onSubsystemAddedEvent(SubsystemAddedEvent event) 
  {
    if(event.getSubsystem().getClass() == CommandSwerveDrivetrain.class)
    {
      this.m_Drivetrain = (CommandSwerveDrivetrain) event.getSubsystem();
      m_Subsystems.unsubscribeSubsystemAdded(this);
    }
  }
  
  @Override
  public void periodic()
  {
    for(Camera camera : enabledCameras)
    {
      camera.updateUnreadPipelines();
      camera.updateMostRecentPipeline();
      updatePositionWithCamera(camera, m_Drivetrain.odometry);
      closestAprilTag = getInRangeTag(camera);
      // enabledCamerasFeed.add(camera.getMostRecentPipeline());

      field.setRobotPose(m_Drivetrain.getPose2d());
      
      SmartDashboard.putNumber("Closest AprilTag ID: ", closestAprilTag.getID());
      SmartDashboard.putNumber("AprilTag Yaw: ", closestAprilTag.getTarget().getYaw());
      SmartDashboard.putData("Robot Pose: ", field);

      

    }
  }
}
