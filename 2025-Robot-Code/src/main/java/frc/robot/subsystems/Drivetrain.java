// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Global;
import frc.robot.constants.DriveConstants;

public class Drivetrain extends LegacySwerveDrivetrain implements Subsystem 
{
  private final Field2d m_Field = new Field2d();
  private Optional<EstimatedRobotPose> optionalPose;
  private PhotonCamera m_CameraRight1, m_CameraLeft1, m_DriverCamera;
  private PhotonPoseEstimator m_CameraRight1Estimator, m_CameraLeft1Estimator;
  private final Timer m_TelemetryTimer = new Timer();
  private final Timer m_ExtraTelemetryTimer = new Timer();
  private final Timer m_ApplicationTimer = new Timer();
  private int m_CameraRight1ExceptionCount, m_CameraLeft1ExceptionCount;
  private boolean m_VisionEnabled = true;
  private boolean m_Aligned = false;

  private Pose2d m_CurrentSpeakerPose;
  private final Rotation2d m_RedOperatorForwardPerspective = Rotation2d.fromDegrees(180);
  private final Rotation2d m_BlueOperatorForwardPerspective = Rotation2d.fromDegrees(0);
  private boolean hasAppliedPerspective = false;
  public int axisModifier = 1;

  private RobotConfig config;


  private final LegacySwerveRequest.ApplyChassisSpeeds m_AutoRequest = new LegacySwerveRequest.ApplyChassisSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic);

  /** 
    @brief Creates a new Drivetrain.
    @param driveTrainConstants Drivetrain-wide constants for the swerve drive
    @param OdometryUpdateFrequency The frequency to run the odometry loop. If unspecified, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0
    @param modules Constants for each specific module 
  */
  public Drivetrain(LegacySwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, LegacySwerveModuleConstants... modules)
  {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    configurePhotonVision();
    configurePathPlanner();

    if(Global.ENABLE_TELEMETRY)
    {
      m_TelemetryTimer.start();
    }
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      m_ExtraTelemetryTimer.start();
    }

    m_ApplicationTimer.start();

  
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  @Override
  public void periodic()
  {
    
    if (m_ApplicationTimer.get() > 5.0)
    {
      if (!hasAppliedPerspective || DriverStation.isDisabled())
      {
        DriverStation.getAlliance().ifPresent((allianceColor) -> {
          this.setOperatorPerspectiveForward
          (
            allianceColor == Alliance.Red ? m_RedOperatorForwardPerspective: m_BlueOperatorForwardPerspective
          );
          hasAppliedPerspective = true;
          
          if(allianceColor == Alliance.Red)
          {
            this.m_CurrentSpeakerPose = DriveConstants.APRIL_TAG_POSES.RED_SPEAKER;
            axisModifier = -1;
          }
          else
          {
            this.m_CurrentSpeakerPose = DriveConstants.APRIL_TAG_POSES.BLUE_SPEAKER;
            axisModifier = 1;
          }
        });
      }

      m_ApplicationTimer.reset();
    }

    m_CameraRight1ExceptionCount = updateVisionWithCamera(m_CameraRight1, m_CameraRight1Estimator, m_CameraRight1ExceptionCount);
    m_CameraLeft1ExceptionCount = updateVisionWithCamera(m_CameraLeft1, m_CameraLeft1Estimator, m_CameraLeft1ExceptionCount);

    if(Global.ENABLE_TELEMETRY)
    {
      if(m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
      {
        m_Field.setRobotPose(m_odometry.getEstimatedPosition());
        SmartDashboard.putData(m_Field);
        m_TelemetryTimer.reset();
      }
    }
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      if(m_ExtraTelemetryTimer.get() > Global.EXTRA_TELEMETRY_UPDATE_SPEED)
      {
        m_ExtraTelemetryTimer.reset();
        SmartDashboard.putBoolean("CameraRight1 isConnected", m_CameraRight1.isConnected());
        SmartDashboard.putBoolean("CameraLeft1 isConnected", m_CameraLeft1.isConnected());
        SmartDashboard.putNumber("CameraRight1ExceptionCount", m_CameraRight1ExceptionCount);
        SmartDashboard.putNumber("CameraLeft1ExceptionCount", m_CameraLeft1ExceptionCount);
        SmartDashboard.putBoolean("Vision Enabled", this.getVisionEnabled());
      }
    } 
  }

  /**
   * @brief Applies a swerve request to the drivetrain
   * @param requestSupplier Supplier for the swerve request
   * @return Applies the given swerve request to the swerve modules
   */
  public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier)
  {
    return run(() -> this.setControl(requestSupplier.get())); 
  }

  public Command applyAutoRequest()
  {
    return run(() -> this.setControl(m_AutoRequest));
  }

  public SwerveDrivePoseEstimator getOdometry()
  {
    return m_odometry;
  }

  private Pose2d getSpeakerPose()
  {
    return m_CurrentSpeakerPose;
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds()
  {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public final Supplier<Rotation2d> yawToSpeaker = () -> this.m_odometry.getEstimatedPosition().getRotation().rotateBy(PhotonUtils.getYawToPose(this.m_odometry.getEstimatedPosition(), this.getSpeakerPose()));

  public final Supplier<Double> distanceToSpeaker = () -> PhotonUtils.getDistanceToPose(this.m_odometry.getEstimatedPosition(), this.getSpeakerPose());
  
  public void setAligned(boolean aligned)
  {
    this.m_Aligned = aligned;
  }

  public boolean getAligned()
  {
    return this.m_Aligned;
  }

  private void configurePhotonVision()
  {
    m_CameraRight1 = new PhotonCamera("CameraRight1");
    m_CameraLeft1 = new PhotonCamera("CameraLeft1");
    m_DriverCamera = new PhotonCamera("DriverCamera");
    m_DriverCamera.setDriverMode(true);

    m_CameraRight1Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, DriveConstants.CAMERA_POSITIONS.RIGHT_1);
    m_CameraLeft1Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, DriveConstants.CAMERA_POSITIONS.LEFT_1);
    m_CameraRight1ExceptionCount = m_CameraLeft1ExceptionCount = 0;
  }

  public boolean getVisionEnabled()
  {
    return this.m_VisionEnabled;
  }

  public void setVisionEnabled(boolean enabled)
  {
    this.m_VisionEnabled = enabled;
  }
  
  /** 
    @brief Configure PathPlanner objects for automatic path following
  */
  private void configurePathPlanner()
  {
    //Determine the radius of the drivebase from module locations
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) 
    {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    
    // Create drivetrain object for pathplanner to use in its calculations
    AutoBuilder.configure(
      ()->this.m_odometry.getEstimatedPosition(),
      this::seedFieldRelative,
      this::getCurrentRobotChassisSpeeds,
      (speeds, ffSpeeds)->this.setControl(m_AutoRequest.withSpeeds(speeds)),
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
      new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
);
      
  }

  private int updateVisionWithCamera(PhotonCamera camera, PhotonPoseEstimator estimator, int exceptionCount)
  {
    if(camera.isConnected())
    {
      optionalPose = estimator.update(null);
      try
      {
        if(optionalPose.isPresent())
        {
          EstimatedRobotPose pose = optionalPose.get();
          this.m_odometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }
      }
      catch(Exception e)
      {
        if(exceptionCount == 0)
        {
          System.out.println("[DRIVE] WARNING: Something bad is happening with the " + camera.getName() + "estimator.");
        }
        return exceptionCount + 1;
      }
    }
    return exceptionCount;
  }
}