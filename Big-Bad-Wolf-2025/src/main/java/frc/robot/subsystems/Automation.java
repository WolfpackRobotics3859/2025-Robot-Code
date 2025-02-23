// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutomationConstants;
import frc.robot.utilities.SubsystemManager;
import frc.robot.utilities.subsystemManager.SubsystemAddedEvent;
import frc.robot.utilities.subsystemManager.SubsystemAddedListener;

public class Automation extends SubsystemBase implements SubsystemAddedListener
{
  private SubsystemManager m_Subsystems;
  private CommandSwerveDrivetrain m_Drivetrain;

  private PhotonCamera m_ForwardCamera;
  private PhotonPoseEstimator m_ForwardCameraEstimator;

  private SwerveRequest.ApplyRobotSpeeds m_SwerveRequest;

  private Field2d m_Field = new Field2d();

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public Automation(SubsystemManager manager) 
  {
    this.m_Subsystems = manager;
    this.m_SwerveRequest = new SwerveRequest.ApplyRobotSpeeds();

    if(this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).isPresent())
    {
      this.m_Drivetrain = this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
      this.Configure();
    }
    else
    {
      m_Subsystems.subscribeSubsystemAdded(this);
    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    this.UpdateForwardCamera();
    SmartDashboard.putData("Field", m_Field);
    m_Field.setRobotPose(this.m_Drivetrain.getState().Pose);
  }

  @Override
  public void onSubsystemAddedEvent(SubsystemAddedEvent event) 
  {
    if(event.getSubsystem().getClass() == CommandSwerveDrivetrain.class)
    {
      this.m_Drivetrain = (CommandSwerveDrivetrain) event.getSubsystem();
      m_Subsystems.unsubscribeSubsystemAdded(this);
      this.Configure();
    }
  }

  private void Configure()
  {
    this.ConfigureCameras();
    this.ConfigureAutobuilder(); // autobuilder should be configured last?
  }

  private void ConfigureAutobuilder()
  {
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();
    } 
    catch (Exception e) 
    {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
            () -> this.m_Drivetrain.getState().Pose, // Robot pose supplier
            (pose) -> this.m_Drivetrain.resetPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
            () -> this.m_Drivetrain.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> this.m_Drivetrain.setControl(m_SwerveRequest.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this.m_Drivetrain
    );
  }

  private void ConfigureCameras()
  {
    this.m_ForwardCamera = new PhotonCamera("FORWARD_CAM");
    this.m_ForwardCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, AutomationConstants.FORWARD_CAMERA_TO_ROBOT);
  }

  private void UpdateForwardCamera()
  {
    if(m_ForwardCamera.isConnected())
    {
      List<PhotonPipelineResult> list = m_ForwardCamera.getAllUnreadResults();
      if(!list.isEmpty())
      {
        if(this.m_Drivetrain != null)
        {
          Optional<EstimatedRobotPose> estimatedPose = m_ForwardCameraEstimator.update(list.get(0));
          if(estimatedPose.isPresent())
          {
            this.m_Drivetrain.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
          }  
        }
        
      }
    }
  }
}