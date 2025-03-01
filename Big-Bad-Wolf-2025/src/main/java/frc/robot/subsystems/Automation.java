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
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutomationConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.utilities.DataSelector;
import frc.robot.utilities.DataStuff;
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

  private boolean m_IsVisionEnabled;
  private boolean m_CleanScheduled;

  private Field2d m_Field = new Field2d();

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private PathPlannerPath ONE_LEFT_ALIGN;
  private PathPlannerPath ONE_RIGHT_ALIGN;
  private PathPlannerPath TWO_LEFT_ALIGN;
  private PathPlannerPath TWO_RIGHT_ALIGN;
  private PathPlannerPath THREE_LEFT_ALIGN;
  private PathPlannerPath THREE_RIGHT_ALIGN;
  private PathPlannerPath FOUR_LEFT_ALIGN;
  private PathPlannerPath FOUR_RIGHT_ALIGN;
  private PathPlannerPath FIVE_LEFT_ALIGN;
  private PathPlannerPath FIVE_RIGHT_ALIGN;
  private PathPlannerPath SIX_LEFT_ALIGN;
  private PathPlannerPath SIX_RIGHT_ALIGN;

  private PathPlannerPath ONE_CLEAN_ALIGN;
  private PathPlannerPath ONE_CLEAN_DEPARTURE;
  private PathPlannerPath TWO_CLEAN_ALIGN;
  private PathPlannerPath TWO_CLEAN_DEPARTURE;
  private PathPlannerPath THREE_CLEAN_ALIGN;
  private PathPlannerPath THREE_CLEAN_DEPARTURE;
  private PathPlannerPath FOUR_CLEAN_ALIGN;
  private PathPlannerPath FOUR_CLEAN_DEPARTURE;
  private PathPlannerPath FIVE_CLEAN_ALIGN;
  private PathPlannerPath FIVE_CLEAN_DEPARTURE;
  private PathPlannerPath SIX_CLEAN_ALIGN;
  private PathPlannerPath SIX_CLEAN_DEPARTURE;

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
      this.Configure();
    }
    else
    {
      m_Subsystems.subscribeSubsystemAdded(this);
    }
  }

  public Command ResetTheStuffs()
  {
    return this.m_Shooter.StopCoral()
                         .andThen(m_Shooter.SetWristPosition(ShooterConstants.WRIST_STOW_POSITION))
                         .andThen(m_Elevator.MoveToLevel(LEVELS.HOME));
  }

  public Command SixRightAlignLevelTwo() 
  {
    return SixRightAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command SixRightAlignPreparationTwo()
  {
    return AutoBuilder.followPath(SIX_RIGHT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }


  public Command OneRightAlignLevelTwo() 
  {
    return OneRightAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command OneRightAlignPreparationTwo()
  {
    return AutoBuilder.followPath(ONE_RIGHT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }


  public Command twoRightAlignLevelTwo() 
  {
    return twoRightAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command twoRightAlignPreparationTwo()
  {
    return AutoBuilder.followPath(TWO_RIGHT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }


  public Command threeRightAlignLevelTwo() 
  {
    return threeRightAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command threeRightAlignPreparationTwo()
  {
    return AutoBuilder.followPath(THREE_RIGHT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }


  public Command fourRightAlignLevelTwo() 
  {
    return fourRightAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command fourRightAlignPreparationTwo()
  {
    return AutoBuilder.followPath(FOUR_RIGHT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }

  public Command fiveRightAlignLevelTwo() 
  {
    return fiveRightAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command fiveRightAlignPreparationTwo()
  {
    return AutoBuilder.followPath(FIVE_RIGHT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }

  public Command SixLeftAlignLevelTwo() 
  {
    return SixLeftAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command SixLeftAlignPreparationTwo()
  {
    return AutoBuilder.followPath(SIX_LEFT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }


  public Command OneLeftAlignLevelTwo() 
  {
    return OneLeftAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command OneLeftAlignPreparationTwo()
  {
    return AutoBuilder.followPath(ONE_LEFT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }

  public Command twoLeftAlignLevelTwo() 
  {
    return twoLeftAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command twoLeftAlignPreparationTwo()
  {
    return AutoBuilder.followPath(TWO_LEFT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }

  public Command threeLeftAlignLevelTwo() 
  {
    return threeLeftAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command threeLeftAlignPreparationTwo()
  {
    return AutoBuilder.followPath(THREE_LEFT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }

  public Command fourLeftAlignLevelTwo() 
  {
    return fourLeftAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command fourLeftAlignPreparationTwo()
  {
    return AutoBuilder.followPath(FOUR_LEFT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }

  public Command fiveLeftAlignLevelTwo() 
  {
    return fiveLeftAlignPreparationTwo().andThen(this.m_Shooter.DeployCoralLowSmiley());
  }

  public Command fiveLeftAlignPreparationTwo()
  {
    return AutoBuilder.followPath(FIVE_LEFT_ALIGN)
                      .alongWith(this.m_Elevator.MoveToLevel(LEVELS.TWO))
                      .alongWith(this.m_Shooter.PrepareToDeployCoralLow());
  }


  public Command oneCleanCoral() {
    return AutoBuilder.followPath(ONE_CLEAN_ALIGN)
    .alongWith(this.m_Elevator.MoveToLevel(LEVELS.HIGH_ALGAE))
    .andThen(m_Shooter.SetWristPosition(ShooterConstants.WRIST_ALGAE_SWEEPING_POSITION))
    .andThen(m_Shooter.SweepAlgae())
    .andThen(AutoBuilder.followPath(ONE_CLEAN_DEPARTURE));
  }

  public Command oneCleanCoralExit() {
    return AutoBuilder.followPath(ONE_CLEAN_DEPARTURE)
    .alongWith(this.m_Elevator.MoveToLevel(LEVELS.HOME))
    .alongWith(this.m_Shooter.StopAlgae());
  }

  public Command CleanAlgae()
  {
    int selectedFace = DataStuff.GetFace();

    PathPlannerPath desiredPath;
    PathPlannerPath desiredDeparturePath;
    boolean isHigh;

    switch(selectedFace)
    {
      case 0:
        desiredPath = this.ONE_CLEAN_ALIGN;
        desiredDeparturePath = this.ONE_CLEAN_DEPARTURE;
        isHigh = false;
      break;

      case 1:
        desiredPath = this.TWO_CLEAN_ALIGN;
        desiredDeparturePath = this.TWO_CLEAN_DEPARTURE;
        isHigh = true;
      break;

      case 2:
        desiredPath = this.THREE_CLEAN_ALIGN;
        desiredDeparturePath = this.THREE_CLEAN_DEPARTURE;
        isHigh = false;
      break;

      case 3:
        desiredPath = this.FOUR_CLEAN_ALIGN;
        desiredDeparturePath = this.FOUR_CLEAN_DEPARTURE;
        isHigh = true;
      break;

      case 4:
        desiredPath = this.FIVE_CLEAN_ALIGN;
        desiredDeparturePath = this.FIVE_CLEAN_DEPARTURE;
        isHigh = false;
      break;

      case 5:
      
      default:
        desiredPath = this.SIX_CLEAN_ALIGN;
        desiredDeparturePath = this.SIX_CLEAN_DEPARTURE;
        isHigh = true;
      break;
    }

    if(isHigh)
    {
      return this.m_Elevator.MoveToLevel(LEVELS.HIGH_ALGAE)
                            .andThen(m_Shooter.SweepAlgae())
                            .andThen(m_Shooter.SetWristPosition(ShooterConstants.WRIST_ALGAE_SWEEPING_POSITION))
                            .andThen(AutoBuilder.followPath(desiredPath))
                            .andThen(m_Shooter.HoldAlgae())
                            .andThen(AutoBuilder.followPath(desiredDeparturePath))
                            .andThen(m_Shooter.SetWristPosition(ShooterConstants.WRIST_STOW_POSITION));
    }

    return this.m_Elevator.MoveToLevel(LEVELS.LOW_ALGAE)
                            .andThen(m_Shooter.SweepAlgae())
                            .andThen(m_Shooter.SetWristPosition(ShooterConstants.WRIST_ALGAE_SWEEPING_POSITION))
                            .andThen(AutoBuilder.followPath(desiredPath))
                            .andThen(m_Shooter.HoldAlgae())
                            .andThen(AutoBuilder.followPath(desiredDeparturePath))
                            .andThen(m_Shooter.SetWristPosition(ShooterConstants.WRIST_STOW_POSITION));
  }

  // public Command DeployToCertainFace()
  // {
  //   int selectedFace = DataStuff.GetFace();
  //   int selectedLeftOrRight = DataStuff.GetSide(); // get from data selector // left is 0


    
  //   switch(selectedFace)
  //   {
  //     case 0:
  //       if(selectedLeftOrRight == 0)
  //       {
  //         desiredPath = this.ONE_LEFT_ALIGN;
  //       }
  //       desiredPath = this.ONE_RIGHT_ALIGN;
  //     break;

  //     case 1:
  //       if(selectedLeftOrRight == 0)
  //       {
  //         desiredPath = this.TWO_LEFT_ALIGN;
  //       }
  //       desiredPath = this.TWO_RIGHT_ALIGN;
  //     break;

  //     case 2:
  //       if(selectedLeftOrRight == 0)
  //       {
  //         desiredPath = this.THREE_LEFT_ALIGN;
  //       }
  //       desiredPath = this.THREE_RIGHT_ALIGN;
  //     break;

  //     case 3:
  //       if(selectedLeftOrRight == 0)
  //       {
  //         desiredPath = this.FOUR_LEFT_ALIGN;
  //       }
  //       desiredPath = this.FOUR_RIGHT_ALIGN;
  //     break;

  //     case 4:
  //       if(selectedLeftOrRight == 0)
  //       {
  //         desiredPath = this.FIVE_LEFT_ALIGN;
  //       }
  //       desiredPath = this.FIVE_RIGHT_ALIGN;
  //     break;

  //     case 5:
      
  //     default:
  //       if(selectedLeftOrRight == 0)
  //       {
  //         desiredPath = this.SIX_LEFT_ALIGN;
  //       }
  //       desiredPath = this.SIX_RIGHT_ALIGN;
  //     break;
  //   }

  // }


  public Command CoralPlacementRoutine()
  {  
    int selectedLevel = DataStuff.GetLevel();
    SmartDashboard.putNumber("Selected Level", selectedLevel);
    LEVELS desiredLevel; 

    switch(selectedLevel)
    {
      case 0:
        desiredLevel = LEVELS.ONE;
      break;

      case 1:
        desiredLevel = LEVELS.TWO;
      break;

      case 2:
        desiredLevel = LEVELS.THREE;
      break;

      case 3:

      default:
        desiredLevel = LEVELS.FOUR;
      break;
    }

    int selectedFace = DataStuff.GetFace(); // Get from DataSelector Later
    int selectedLeftOrRight = DataStuff.GetSide(); // get from data selector // left is 0
    PathPlannerPath desiredPath;

    SmartDashboard.putNumber("Selected Face", selectedFace);
    SmartDashboard.putNumber("Selected LR", selectedLeftOrRight);

    switch(selectedFace)
    {
      case 0:
        if(selectedLeftOrRight == 0)
        {
          desiredPath = this.ONE_LEFT_ALIGN;
        }
        desiredPath = this.ONE_RIGHT_ALIGN;
      break;

      case 1:
        if(selectedLeftOrRight == 0)
        {
          desiredPath = this.TWO_LEFT_ALIGN;
        }
        desiredPath = this.TWO_RIGHT_ALIGN;
      break;

      case 2:
        if(selectedLeftOrRight == 0)
        {
          desiredPath = this.THREE_LEFT_ALIGN;
        }
        desiredPath = this.THREE_RIGHT_ALIGN;
      break;

      case 3:
        if(selectedLeftOrRight == 0)
        {
          desiredPath = this.FOUR_LEFT_ALIGN;
        }
        desiredPath = this.FOUR_RIGHT_ALIGN;
      break;

      case 4:
        if(selectedLeftOrRight == 0)
        {
          desiredPath = this.FIVE_LEFT_ALIGN;
        }
        desiredPath = this.FIVE_RIGHT_ALIGN;
      break;

      case 5:
      
      default:
        if(selectedLeftOrRight == 0)
        {
          desiredPath = this.SIX_LEFT_ALIGN;
        }
        desiredPath = this.SIX_RIGHT_ALIGN;
      break;
    }

    if(this.m_IsVisionEnabled)
    {
      if(desiredLevel == LEVELS.FOUR)
      {
        return AutoBuilder.followPath(desiredPath)
                        .andThen(this.m_Elevator.MoveToLevel(desiredLevel))
                        .andThen(this.m_Shooter.SetWristPosition(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION))
                        .andThen(this.m_Shooter.DeployCoralHigh());
      }
      else
      {
        return AutoBuilder.followPath(desiredPath)
                        .andThen(this.m_Elevator.MoveToLevel(desiredLevel))
                        .andThen(this.m_Shooter.SetWristPosition(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION_LOW))
                        .andThen(this.m_Shooter.DeployCoralLow());
      }
    }
    else
    {
      if(desiredLevel == LEVELS.FOUR)
      {
        return this.m_Elevator.MoveToLevel(desiredLevel)
                              .andThen(this.m_Shooter.SetWristPosition(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION))
                              .andThen(this.m_Shooter.DeployCoralHigh());
      }
      else
      {
        return this.m_Elevator.MoveToLevel(desiredLevel)
                              .andThen(this.m_Shooter.SetWristPosition(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION_LOW))
                              .andThen(this.m_Shooter.DeployCoralLow());
      }
    }
  }

  public Command ToggleVision()
  {
    return this.runOnce(() -> this.ToggleVisionAndUpdateSmartdashboard());
  }

  public Command ScheduleCleaning()
  {
    return this.runOnce(() -> this.ScheduleACleaning());
  }

  @Override
  public void periodic() 
  {
    this.UpdateForwardCamera();
    SmartDashboard.putData("Field", m_Field);
    SmartDashboard.putBoolean("Forward Camera Connected", this.m_ForwardCamera.isConnected());
    m_Field.setRobotPose(this.m_Drivetrain.getState().Pose);
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

    // if(event.getSubsystem().getClass() == CommandSwerveDrivetrain.class)
    // {
    //   this.m_Drivetrain = (CommandSwerveDrivetrain) event.getSubsystem();
    //   m_Subsystems.unsubscribeSubsystemAdded(this);
    //   this.Configure();
    // }
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
    this.CachePathsAndAutos();

    m_IsVisionEnabled = true;
    m_CleanScheduled = false;
    SmartDashboard.putBoolean("Clean Scheduled", m_CleanScheduled);
    SmartDashboard.putBoolean("Vision Enabled", m_IsVisionEnabled);
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
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
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

  private void ToggleVisionAndUpdateSmartdashboard()
  {
    this.m_IsVisionEnabled = !this.m_IsVisionEnabled;
    SmartDashboard.putBoolean("Vision Enabled", m_IsVisionEnabled);
  }

  private void ScheduleACleaning()
  {
    this.m_CleanScheduled = !this.m_CleanScheduled;
    SmartDashboard.putBoolean("CLEAN", m_CleanScheduled);
  }

  private void CachePathsAndAutos()
  {
    try
    {
        ONE_LEFT_ALIGN = PathPlannerPath.fromPathFile("ONE-LEFT-ALIGN");
        ONE_RIGHT_ALIGN = PathPlannerPath.fromPathFile("ONE-RIGHT-ALIGN");
        TWO_LEFT_ALIGN = PathPlannerPath.fromPathFile("TWO-LEFT-ALIGN");
        TWO_RIGHT_ALIGN = PathPlannerPath.fromPathFile("TWO-RIGHT-ALIGN");
        THREE_LEFT_ALIGN = PathPlannerPath.fromPathFile("THREE-LEFT-ALIGN");
        THREE_RIGHT_ALIGN = PathPlannerPath.fromPathFile("THREE-RIGHT-ALIGN");
        FOUR_LEFT_ALIGN = PathPlannerPath.fromPathFile("FOUR-LEFT-ALIGN");
        FOUR_RIGHT_ALIGN = PathPlannerPath.fromPathFile("FOUR-RIGHT-ALIGN");
        FIVE_LEFT_ALIGN = PathPlannerPath.fromPathFile("FIVE-LEFT-ALIGN");
        FIVE_RIGHT_ALIGN = PathPlannerPath.fromPathFile("FIVE-RIGHT-ALIGN");
        SIX_LEFT_ALIGN = PathPlannerPath.fromPathFile("SIX-LEFT-ALIGN");
        SIX_RIGHT_ALIGN = PathPlannerPath.fromPathFile("SIX-RIGHT-ALIGN");

        ONE_CLEAN_ALIGN = PathPlannerPath.fromPathFile("ONE-CLEAN-ALIGN");
        ONE_CLEAN_DEPARTURE = PathPlannerPath.fromPathFile("ONE-CLEAN-DEPARTURE");
        TWO_CLEAN_ALIGN = PathPlannerPath.fromPathFile("TWO-CLEAN-ALIGN");
        TWO_CLEAN_DEPARTURE = PathPlannerPath.fromPathFile("TWO-CLEAN-DEPARTURE");
        THREE_CLEAN_ALIGN = PathPlannerPath.fromPathFile("THREE-CLEAN-ALIGN");
        THREE_CLEAN_DEPARTURE = PathPlannerPath.fromPathFile("THREE-CLEAN-DEPARTURE");
        FOUR_CLEAN_ALIGN = PathPlannerPath.fromPathFile("FOUR-CLEAN-ALIGN");
        FOUR_CLEAN_DEPARTURE = PathPlannerPath.fromPathFile("FOUR-CLEAN-DEPARTURE");
        FIVE_CLEAN_ALIGN = PathPlannerPath.fromPathFile("FIVE-CLEAN-ALIGN");
        FIVE_CLEAN_DEPARTURE = PathPlannerPath.fromPathFile("FIVE-CLEAN-DEPARTURE");
        SIX_CLEAN_ALIGN = PathPlannerPath.fromPathFile("SIX-CLEAN-ALIGN");
        SIX_CLEAN_DEPARTURE = PathPlannerPath.fromPathFile("SIX-CLEAN-DEPARTURE");

        DataLogManager.log("Automation has successfully cached pathplanner autos and paths.");
    } 
    catch (Exception e) 
    {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        DataLogManager.log("Automation has failed in caching pathplanner autos and paths.");
    }
  }
}