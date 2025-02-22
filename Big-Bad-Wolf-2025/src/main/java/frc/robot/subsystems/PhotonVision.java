// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

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
  private EstimatedRobotPose visionPose;

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

  /**
   * Creates a new PhotonVision object
   * @param manager SubsystemManager used to access drivetrain
   */
  public PhotonVision(SubsystemManager manager)
  {
    this.m_Subsystems = manager;
    this.m_Drivetrain = this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    // this.m_Drivetrain = p_Drivetrain;
   
    enabledCameras = new ArrayList<>();

    // m_FrontCamera = PhotonConstants.frontCamera;
    // m_BackCamera = PhotonConstants.backCamera;
    m_RightCamera = PhotonConstants.rightCamera;
    // m_LeftCamera = PhotonConstants.leftCamera;    
    // enabledCameras.add(m_FrontCamera);
    // enabledCameras.add(m_BackCamera);
    enabledCameras.add(m_RightCamera);
    // enabledCameras.add(m_LeftCamera);

    taskRange = 1.15; // In meters supposedly
    closestAprilTag = new AprilTagInfo(-1);
    desiredAprilTagId = -1;
    field = new Field2d();

    xyController = new PIDController(0.005, 0, 0.001); 
    angularController = new PIDController(1, 0, 0); 

    xyController.setTolerance(0.1);
    angularController.enableContinuousInput(-Math.PI, Math.PI);
    angularController.setTolerance(0.05);
  }

  /**
   * Takes angle as input and gets the positive coterminal
   * @param angle inputted to perform on
   * @return the coterminal of inputted angle
   */
  public double getCoterminalAngle(double angle)
  {
    return ((angle % 360) + 360) % 360; // Add 360 to find positive coterminal angle then % to get rid of values over 360
  }

  /**
   * Sets the desired id of the PhotonVision object
   * @param id target id to set to
   */
  public void setDesiredId(int id)
  {
    desiredAprilTagId = id;
  }

  /**
   * Gets the closest visible tag to the bot
   * @return closest visible apriltag
   */
  public AprilTagInfo getClosestTag()
  {
    return closestAprilTag;
  }

  /**
   * Gets the apriltag's transform3d
   * @param info inputted apriltag to get transform3d from
   * @return the apriltag's transform3d
   */
  public final Transform3d getTargetTransform3d(AprilTagInfo info)
    {
      return info.getCameraWitness()
                 .getCamera3DPosition()
                 .plus(info.getTarget().getBestCameraToTarget());
    }


    /**
     * Estimates the position of the robot using vision
     * @param camera selected camera to use to update robot position
     * @return the estimated position of robot as EstimatedRobotPose
     */
  public EstimatedRobotPose positionEstimation(Camera camera)
  {
    // Creates an estimated position off most recent pipeline using camera pose estimator
    PhotonPipelineResult pipeline = camera.getMostRecentPipeline();
    if (pipeline == null || !pipeline.hasTargets()) return null;

    return camera.getCameraPoseEstimator()
                 .update(camera.getMostRecentPipeline())
                 .orElse(null);
  }
  
  
  /** 
   * Uses a camera to update the odometry and robot position
   * @param camera The camera to estimate the pose of the robot
   * @param m_odometry The odometry object to send EstimatedRobotPose to
   */
  public final void updatePositionWithCamera(Camera camera, SwerveDrivePoseEstimator m_odometry) 
  {
    // Returns if the camera is not connected
    if (!camera.getPhotonCamera().isConnected()) 
    {
      System.out.println(camera.getCameraName() + " is not connected");
      return;
    }

    // try/catch statement to handle any errors from the camera
    try 
    {    
      visionPose = positionEstimation(camera);

      if (visionPose != null)
      {
      m_odometry.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds); // Adds estimated robot pose to odometry
      } 
      else
      {
        System.out.println("Vision pose not present; Skipping vision update.");
      }
    } 
    catch (Exception e) 
    {
      // Prints out the error of the camera and increments the camera's exception count
      System.out.println("ERROR! \tSource: " + camera.getCameraName() + "\tException: " + e);
      camera.setCameraExceptionCount(camera.getCameraExceptionCount() + 1);
    }
  }
  

  /**
   * Finds visible apriltags that are in the correct task range
   * @param camera selected camera to grab vision from
   * @return returns the apriltag found if one is in range (returns invalid tag else)
   */
  private AprilTagInfo getInRangeTag(Camera camera)
  {
    AprilTagInfo info = new AprilTagInfo(-1); // new instance of AprilTagInfo

    PhotonPipelineResult cameraReading = camera.getMostRecentPipeline();
    if (cameraReading == null) 
    {
      System.out.println("Pipline == null for " + camera.getCameraName());
      return info;
    }

    // Checks if the camera sees an apriltag
    if (!cameraReading.hasTargets()) return info;
    System.out.println("AprilTag detected");

    Pose2d robotPosition = m_Drivetrain.getPose2d();
    PhotonTrackedTarget target = cameraReading.getTargets()
                                              .get(cameraReading.getTargets().size()-1);

    double targetDistance = target.getBestCameraToTarget()
                                  .plus(camera.getCamera3DPosition())
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
  

/** 
 * Checks if the closestAprilTag and desiredAprilTagId matches with closestAprilTagId 
 * @return true if requirments are met, false else
 */
  public final boolean aprilTagTaskReady() 
  {
    // checks if the apriltag is valid
    if (!closestAprilTag.isValid()) return false;
    if (desiredAprilTagId == closestAprilTag.getID()) return true;
    return false;
  }

  /**
   * Aligns the robot with an apriltag (MUST BE LOOKING AT APRILTAG FIRST)
   * @param aprilTag selected apriltag to align to
   */
  public final void align(AprilTagInfo aprilTag) 
  {
    Pose2d currentRobotPosition2d = m_Drivetrain.getPose2d(); // Grabs current pose2d of bot
    Translation2d aprilTagTranslation2d = getTargetTransform3d(aprilTag) // Gets transform3d of apriltag and turns it to translation2d
                                          .getTranslation()
                                          .toTranslation2d();

    Rotation2d aprilTagFacingAwayAngle = aprilTag.getTarget() // Gets the rotation2d of april tag
                                                 .getBestCameraToTarget()
                                                 .getRotation()
                                                 .toRotation2d();

    Translation2d alignedTranslation2d = currentRobotPosition2d.getTranslation() // Creates new aligned translation2d calculated by rotating robot's translations2d around apriltag translations2d by apriltag's rotation
                                                               .rotateAround(aprilTagTranslation2d, aprilTagFacingAwayAngle);
    
    // Gets the compnonets of aligned tranlsation2d
    double alignedX = alignedTranslation2d.getX();
    double alignedY = alignedTranslation2d.getY();

    // Calculates chassis speeds from robot's position components and aligned tranlsation components
    double xCorrection = xyController.calculate(currentRobotPosition2d.getX(), alignedX);
    double yCorrection = xyController.calculate(currentRobotPosition2d.getY(), alignedY);
    
    double angularCorrection = Units.degreesToRadians(angularController.calculate(Units.degreesToRadians(getCoterminalAngle(m_Drivetrain.getPigeon2() // Calculates chassis speeds from current robot angle and aligned angle
                                                                                                                                        .getRotation2d()
                                                                                                                                        .getDegrees())),
                                                                                  aprilTagFacingAwayAngle.getRadians()));
                                              
    // Creates chassis speeds and drives robot using them
    ChassisSpeeds speeds = new ChassisSpeeds(xCorrection, yCorrection, angularCorrection); 
    m_Drivetrain.driveRobotRelative(speeds);
  }

  /**
   * Rotates bot to be looking direclty at april tag
   * @param aprilTag selected apriltag to rotate to
   */
  public final void rotateToAprilTag(AprilTagInfo aprilTag) 
  {
    double tagYawOffset = Units.degreesToRadians(aprilTag.getTarget().getYaw()); // Get the yaw of the AprilTag (degrees from the center of camera)

    double robotYawRaw = m_Drivetrain.getPigeon2() // Use Pigeon2 sensor to get the robot's raw yaw in degrees
                                     .getRotation2d()
                                     .getDegrees();

    double robotYaw = Units.degreesToRadians(getCoterminalAngle(robotYawRaw)); // Gets coterminal angle and converts to radians

    double angularCorrection = angularController.calculate(robotYaw, tagYawOffset); // Calculates chassis speeds to move to apriltag yaw

    // Creates new chassis speeds and drives robot using them
    ChassisSpeeds speeds = new ChassisSpeeds(0,0, angularCorrection);
    m_Drivetrain.driveRobotRelative(speeds);
  }
  
  @Override
  /**
   * runs every 20 ms
   */
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
      if (closestAprilTag.isValid()) 
      {
        SmartDashboard.putNumber("AprilTag Yaw: ", closestAprilTag.getTarget().getYaw());
        SmartDashboard.putNumber(getName(), desiredAprilTagId);
      }
      
      SmartDashboard.putData("Robot Pose: ", field);
      SmartDashboard.putBoolean("Camera_Front Connection Status: ", camera.getPhotonCamera().isConnected());
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
}
