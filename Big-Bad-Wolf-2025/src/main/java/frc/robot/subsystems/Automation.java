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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutomationConstants;
import frc.robot.utilities.SubsystemManager;
import frc.robot.utilities.subsystemManager.SubsystemAddedEvent;
import frc.robot.utilities.subsystemManager.SubsystemAddedListener;

public class Automation extends SubsystemBase implements SubsystemAddedListener
{
  private SubsystemManager m_Subsystems;
  private CommandSwerveDrivetrain m_Drivetrain;
  private Shooter m_Shooter;
  private Elevator m_Elevator;

  private PhotonCamera m_ForwardCamera;
  private PhotonPoseEstimator m_ForwardCameraEstimator;

  private SwerveRequest.ApplyRobotSpeeds m_SwerveRequest;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Current Robot Pose", Pose2d.struct).publish();

  public Automation(SubsystemManager manager) 
  {
    this.m_Subsystems = manager;
    this.m_SwerveRequest = new SwerveRequest.ApplyRobotSpeeds();

    if(this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).isPresent())
    {
      this.m_Drivetrain = this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    }

    if(this.m_Subsystems.getSubsystemOfType(Shooter.class).isPresent())
    {
      this.m_Shooter = this.m_Subsystems.getSubsystemOfType(Shooter.class).get();
    }

    if(this.m_Subsystems.getSubsystemOfType(Elevator.class).isPresent())
    {
      this.m_Elevator = this.m_Subsystems.getSubsystemOfType(Elevator.class).get();
    }

    if((this.m_Drivetrain != null) && (this.m_Shooter != null) && (this.m_Elevator != null))
    {
      // this.Configure();
    }
    else
    {
      m_Subsystems.subscribeSubsystemAdded(this);
    }
    this.Configure();

    publisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
  }

  @Override
  public void periodic() 
  {
    this.UpdateForwardCamera();
    SmartDashboard.putBoolean("Forward Camera Connected", this.m_ForwardCamera.isConnected());
    publisher.set(this.m_Drivetrain.getState().Pose);
  }

  @Override
  public void onSubsystemAddedEvent(SubsystemAddedEvent event) 
  {
    if(this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).isPresent())
    {
      this.m_Drivetrain = this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    }

    if(this.m_Subsystems.getSubsystemOfType(Shooter.class).isPresent())
    {
      this.m_Shooter = this.m_Subsystems.getSubsystemOfType(Shooter.class).get();
    }

    if(this.m_Subsystems.getSubsystemOfType(Elevator.class).isPresent())
    {
      this.m_Elevator = this.m_Subsystems.getSubsystemOfType(Elevator.class).get();
    }

    if((this.m_Drivetrain != null) && (this.m_Shooter != null) && (this.m_Elevator != null))
    {
      m_Subsystems.unsubscribeSubsystemAdded(this);
      this.Configure();
    }
  }

  public Command PathfindToPose(Pose2d goalPose)
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
          3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
          goalPose,
          constraints,
          0.0 // Goal end velocity in meters/sec
    );
  }

  public Command PathfindThenFollowPath(PathPlannerPath path)
  {
    PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  private void Configure()
  {
    this.ConfigureCameras();
    this.ConfigureAutobuilder(); // autobuilder should be configured last?

    System.out.println("AUTOMATION CONFIGURATION COMPLETE.");
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
                    new PIDConstants(10, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent())
              {
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
    this.m_ForwardCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, AutomationConstants.FORWARD_CAMERA_TO_ROBOT);
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