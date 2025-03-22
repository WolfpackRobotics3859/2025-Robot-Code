package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.CameraManager;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.PackLog;
import frc.robot.utilities.SubsystemManager;
import frc.robot.utilities.subsystemManager.SubsystemAddedEvent;
import frc.robot.utilities.subsystemManager.SubsystemAddedListener;

public class Drivetrain extends CommandSwerveDrivetrain implements SubsystemAddedListener
{
    private PackLog m_PackLog;

    private CameraManager m_CameraManager;

    private PIDController m_XController;
    private PIDController m_YController;
    private PIDController m_RotationController;

    private SwerveRequest.FieldCentric m_SwerveRequestField;
    private SwerveRequest.RobotCentric m_SwerveRequestRobot;
    private SwerveRequest.ApplyRobotSpeeds m_SwerveRequestSpeeds;

    private double m_DefaultDriveMaxSpeed;
    private double m_DefaultDriveMaxAngularRate;

    private SubsystemManager m_SubsystemManager;
    private Elevator m_Elevator;

    private final SwerveRequest.FieldCentric m_OperatorDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private PhotonCamera m_ForwardCamera;
   // private PhotonCamera m_FarCamera;
    private PhotonPoseEstimator m_ForwardCameraEstimator;
 //   private PhotonPoseEstimator m_FarCameraEstimator;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

    public Drivetrain(SubsystemManager manager, SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(constants, modules);    
        this.ConfigureDrivetrain();
        this.ConfigureCameras();

        m_DefaultDriveMaxSpeed = TunerConstants.MaxSpeed;
        m_DefaultDriveMaxAngularRate = TunerConstants.MaxAngularRate;

        m_SubsystemManager = manager;

        if (m_SubsystemManager.getSubsystemOfType(Elevator.class).isPresent())
        {
            m_Elevator = m_SubsystemManager.getSubsystemOfType(Elevator.class).get();
        }
        else
        {
            m_SubsystemManager.subscribeSubsystemAdded(this);
        }
    }

    @Override
    public void periodic() 
    {
        updateDefaultDriveSpeeds();
        super.periodic();
  //      this.m_CameraManager.UpdateCameras();
        this.UpdateForwardCamera();
   //     this.UpdateFarCamera();
        SmartDashboard.putBoolean("Forward Camera Connected", this.m_ForwardCamera.isConnected());
    //    SmartDashboard.putBoolean("Far Camera Connected", this.m_FarCamera.isConnected());
        publisher.set(this.getState().Pose);
    }

    private void ConfigureCameras()
    {
        this.m_ForwardCamera = new PhotonCamera("FORWARD_CAM");
        this.m_ForwardCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.ROBOT_TO_CAM_TRANFORMS[0]);
        // this.m_FarCamera = new PhotonCamera("FAR_CAM");
        // this.m_FarCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.ROBOT_TO_CAM_TRANFORMS[2]);
    }

    private void UpdateForwardCamera()
    {
    if(m_ForwardCamera.isConnected())
    {
        List<PhotonPipelineResult> list = m_ForwardCamera.getAllUnreadResults();
        if(!list.isEmpty())
        {
            Optional<EstimatedRobotPose> estimatedPose = m_ForwardCameraEstimator.update(list.get(0));
            if(estimatedPose.isPresent())
            {
            this.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
            }  
        }
    }
    }

    // private void UpdateFarCamera()
    // {
    // if(m_FarCamera.isConnected())
    // {
    //     List<PhotonPipelineResult> list = m_FarCamera.getAllUnreadResults();
    //     if(!list.isEmpty())
    //     {
    //         Optional<EstimatedRobotPose> estimatedPose = m_FarCameraEstimator.update(list.get(0));
    //         if(estimatedPose.isPresent())
    //         {
    //         this.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
    //         }  
    //     }
    // }
    // }

    public Command AlignCoral()
    {
        return new FunctionalCommand(() -> 
                                        {
                                            Pose2d goalPose = this.GetGoalCoralPose();
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                            this.m_RotationController.setSetpoint(goalPose.getRotation().getDegrees());
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> IsAlignmentComplete(), 
                                     this);
    }

    public Command AlignCoralNoEnd()
    {
        return new FunctionalCommand(() -> 
                                        {
                                            Pose2d goalPose = this.GetGoalCoralPose();
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                            this.m_RotationController.setSetpoint(goalPose.getRotation().getDegrees());
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {}, 
                                     () -> false, 
                                     this);
    }

    public Command AlignToFace(int side, int face)
    {
        return new FunctionalCommand(() -> 
                                        {
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            Pose2d goalPose;
                                            if(alliance == Alliance.Blue)
                                            {
                                                if(side == 0)
                                                {
                                                    goalPose = DrivetrainConstants.BLUE_LEFT_ALIGNMENTS[face - 1];
                                                }
                                                else
                                                {
                                                    goalPose = DrivetrainConstants.BLUE_RIGHT_ALIGNMENTS[face - 1];
                                                }
                                            }
                                            else
                                            {
                                                if(side == 0)
                                                {
                                                    goalPose = DrivetrainConstants.RED_LEFT_ALIGNMENTS[face - 1];
                                                }
                                                else
                                                {
                                                    goalPose = DrivetrainConstants.RED_RIGHT_ALIGNMENTS[face -1];
                                                }
                                            }
                                            
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                            this.m_RotationController.setSetpoint(goalPose.getRotation().getDegrees());
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {}, 
                                     () -> false, 
                                     this);
    }

    public Command AlignCenter()
    {
        return new FunctionalCommand(() -> 
                                        {
                                            Pose2d goalPose = this.GetGoalCleanPose();
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                            this.m_RotationController.setSetpoint(goalPose.getRotation().getDegrees());
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> IsAlignmentComplete(), 
                                     this);
    }

    public Command AlignCenterNoEnd()
    {
        return new FunctionalCommand(() -> 
                                        {
                                            Pose2d goalPose = this.GetGoalCleanPose();
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                            this.m_RotationController.setSetpoint(goalPose.getRotation().getDegrees());
                                        }, 
                                     () -> UpdateRequest(),
                                     interrupted -> {}, 
                                     () -> false, 
                                     this);
    }



    private Pose2d GetGoalCoralPose()
    {
        if(alliance == Alliance.Blue)
        {
            if(DataStuff.GetCurrentSide() == 0)
            {
                return DrivetrainConstants.BLUE_LEFT_ALIGNMENTS[DataStuff.GetCurrentFace()];
            }
            return DrivetrainConstants.BLUE_RIGHT_ALIGNMENTS[DataStuff.GetCurrentFace()];
        }

        if(DataStuff.GetCurrentSide() == 0)
        {
            return DrivetrainConstants.RED_LEFT_ALIGNMENTS[DataStuff.GetCurrentFace()];
        }
        return DrivetrainConstants.RED_RIGHT_ALIGNMENTS[DataStuff.GetCurrentFace()];
    }

    private Pose2d GetGoalCleanPose()
    {
        if(alliance == Alliance.Blue)
        {
            return DrivetrainConstants.BLUE_CENTER_ALIGNMENTS[DataStuff.GetCurrentFace()];
        }
        return DrivetrainConstants.RED_CENTER_ALIGNMENTS[DataStuff.GetCurrentFace()];
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

    public Command DefaultDrive(Supplier<Double> thrust, Supplier<Double> strafe, Supplier<Double> rotation)
    {
        return this.applyRequest(() -> this.m_OperatorDriveRequest.withVelocityX(-thrust.get() * m_DefaultDriveMaxSpeed * 0.8)
                                                                  .withVelocityY(-strafe.get() * m_DefaultDriveMaxSpeed * 0.8)
                                                                  .withRotationalRate(-rotation.get() * m_DefaultDriveMaxAngularRate));
    }

    private void UpdateRequest()
    {
        if(alliance == Alliance.Red)
        {
            this.m_SwerveRequestField.VelocityX = -this.GetXOutput();
            this.m_SwerveRequestField.VelocityY = -this.GetYOutput();
            this.m_SwerveRequestField.RotationalRate = this.GetRotationOutput();
            this.setControl(m_SwerveRequestField);
        }
        else
        {
            this.m_SwerveRequestField.VelocityX = this.GetXOutput();
            this.m_SwerveRequestField.VelocityY = this.GetYOutput();
            this.m_SwerveRequestField.RotationalRate = this.GetRotationOutput();
            this.setControl(m_SwerveRequestField);
        }
    }

    private double GetXOutput()
    {
        return MathUtil.clamp(this.m_XController.calculate(this.getState().Pose.getX()), -TunerConstants.MaxSpeed * 0.5, TunerConstants.MaxSpeed * 0.5);
    }

    private double GetYOutput()
    {
        return MathUtil.clamp(this.m_YController.calculate(this.getState().Pose.getY()), -TunerConstants.MaxSpeed * 0.5, TunerConstants.MaxSpeed * 0.5);
    }

    private double GetRotationOutput()
    {
        return MathUtil.clamp(this.m_RotationController.calculate(this.getState().Pose.getRotation().getDegrees()), -TunerConstants.MaxAngularRate, TunerConstants.MaxAngularRate);
    }

    private boolean IsAlignmentComplete()
    {
        return this.m_XController.atSetpoint() && this.m_YController.atSetpoint();
    }

    private void updateDefaultDriveSpeeds()
    {
        double elevatorPosition = m_Elevator.getElevatorPosition();
        // SmartDashboard.putNumber("elevator position!!: ", m_Elevator.getElevatorPosition());
        double threshold = LEVELS.HOME.getValue();
        double scale;
        if (elevatorPosition <= threshold)
        {
            scale = 1;
        }
        else
        {
            scale = 1 - ((elevatorPosition - threshold) / (LEVELS.MAX_HIEGHT.getValue() - threshold));
            scale = (scale < 0.2) ? 0.2 : scale;

            // SmartDashboard.putNumber("Scale!!!: ", scale);
        }
        m_DefaultDriveMaxAngularRate = (TunerConstants.MaxAngularRate * scale);
        m_DefaultDriveMaxSpeed = (TunerConstants.MaxSpeed * scale);
    }

    private void ConfigureDrivetrain()
    {
        this.m_PackLog = new PackLog("Drivetrain");
        this.ConfigureAutobuilder();
        this.ConfigurePIDControllers();

        this.m_CameraManager = new CameraManager(this);
        this.m_CameraManager.InitializeCameras(CameraConstants.CAMERA_PIPELINES, CameraConstants.ROBOT_TO_CAM_TRANFORMS);

        this.m_SwerveRequestField = new SwerveRequest.FieldCentric();
        this.m_SwerveRequestRobot = new SwerveRequest.RobotCentric();
        this.m_SwerveRequestSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        this.m_PackLog.Log("End configuration.");
    }

    private void ConfigurePIDControllers()
    {
        this.m_PackLog.Log("Beginning configuration.");
        this.m_XController = new PIDController(15, 0.0,0.025); // 20 0.1 0.025
        this.m_XController.setTolerance(0.01, 0.025);
        this.m_XController.setIntegratorRange(-TunerConstants.MaxSpeed * 0.1,TunerConstants.MaxSpeed * 0.1);
        this.m_XController.setIZone(0.01);
        SmartDashboard.putData(this.m_XController);

        this.m_YController = new PIDController (15, 0.0 ,0.025);
        this.m_YController.setTolerance(0.01, 0.025);
        this.m_YController.setIntegratorRange(-TunerConstants.MaxSpeed * 0.1,TunerConstants.MaxSpeed * 0.1);
        this.m_YController.setIZone(0.01);
        SmartDashboard.putData(this.m_YController);

        this.m_RotationController = new PIDController (0.3, 0.0 ,0); // 0.25 
        this.m_RotationController.setTolerance(1, 1);
        this.m_RotationController.setIntegratorRange(-TunerConstants.MaxAngularRate * 0.1,TunerConstants.MaxAngularRate * 0.1);
        this.m_RotationController.setIZone(10);
        this.m_RotationController.enableContinuousInput(-180, 180);
        SmartDashboard.putData(this.m_RotationController);
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
                () -> this.getState().Pose, // Robot pose supplier
                (pose) -> this.resetPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> this.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.setControl(m_SwerveRequestSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(15, 0.0, 0.0), // Translation PID constants
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
                this
        );
    }

    @Override
  public void onSubsystemAddedEvent(SubsystemAddedEvent event) 
  {
    if (event.getSubsystem().getClass() == Elevator.class)
    {
      m_Elevator = (Elevator)event.getSubsystem();
      m_SubsystemManager.unsubscribeSubsystemAdded(this);
    }
  }
}

