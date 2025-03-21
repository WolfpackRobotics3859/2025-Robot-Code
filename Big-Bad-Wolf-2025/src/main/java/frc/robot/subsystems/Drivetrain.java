package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.PackLog;

public class Drivetrain extends CommandSwerveDrivetrain 
{
    private PackLog m_PackLog;

    private PIDController m_XController;
    private PIDController m_YController;

    private SwerveRequest.FieldCentricFacingAngle m_SwerveFieldCentricFacingAngle;
    private SwerveRequest.RobotCentric m_SwerveRequestRobot;
    private SwerveRequest.ApplyRobotSpeeds m_SwerveRequestSpeeds;

    private final SwerveRequest.FieldCentric m_OperatorDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private PhotonCamera m_ForwardCamera;
    private PhotonCamera m_FarCamera;
    private PhotonPoseEstimator m_ForwardCameraEstimator;
    private PhotonPoseEstimator m_FarCameraEstimator;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

    public Drivetrain(SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(constants, modules);    
        this.ConfigureDrivetrain();
        this.ConfigureCameras();
    }

    @Override
    public void periodic() 
    {
        super.periodic();
        this.UpdateForwardCamera();
        this.UpdateFarCamera();
        publisher.set(this.getState().Pose);
    }

    private void ConfigureCameras()
    {
        this.m_ForwardCamera = new PhotonCamera("FORWARD_CAM");
        this.m_ForwardCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.ROBOT_TO_CAM_TRANFORMS[0]);
        this.m_FarCamera = new PhotonCamera("FAR_CAM");
        this.m_FarCameraEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.ROBOT_TO_CAM_TRANFORMS[2]);
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

    private void UpdateFarCamera()
    {
    if(m_FarCamera.isConnected())
    {
        List<PhotonPipelineResult> list = m_FarCamera.getAllUnreadResults();
        if(!list.isEmpty())
        {
            Optional<EstimatedRobotPose> estimatedPose = m_FarCameraEstimator.update(list.get(0));
            if(estimatedPose.isPresent())
            {
            this.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
            }  
        }
    }
    }

    public Command AlignCoral()
    {
        return new FunctionalCommand(() -> 
                                        {
                                            Pose2d goalPose = this.GetGoalCoralPose();
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                            this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation().rotateBy(Rotation2d.k180deg);
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
                                            this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation().rotateBy(Rotation2d.k180deg);
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
                                            this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation();
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
                                            Pose2d goalPose = this.GetGoalCoralPose();
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> IsAlignmentComplete(), 
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
            1.0, 4.0,
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
        return this.applyRequest(() -> this.m_OperatorDriveRequest.withVelocityX(-thrust.get() * TunerConstants.MaxSpeed * 0.8)
                                                                  .withVelocityY(-strafe.get() * TunerConstants.MaxSpeed * 0.8)
                                                                  .withRotationalRate(-rotation.get() * TunerConstants.MaxAngularRate));
    }

    private void UpdateRequest()
    {
        if(alliance == Alliance.Red)
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = -this.GetXOutput();
            this.m_SwerveFieldCentricFacingAngle.VelocityY = -this.GetYOutput();
            this.setControl(m_SwerveFieldCentricFacingAngle);
        }
        else
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = this.GetXOutput();
            this.m_SwerveFieldCentricFacingAngle.VelocityY = this.GetYOutput();
            this.setControl(m_SwerveFieldCentricFacingAngle);
        }
    }

    private double GetXOutput()
    {
        return MathUtil.clamp(this.m_XController.calculate(this.getState().Pose.getX()), -TunerConstants.MaxSpeed, TunerConstants.MaxSpeed);
    }

    private double GetYOutput()
    {
        return MathUtil.clamp(this.m_YController.calculate(this.getState().Pose.getY()), -TunerConstants.MaxSpeed, TunerConstants.MaxSpeed);
    }

    Debouncer debounce = new Debouncer(0.25, DebounceType.kRising);

    private boolean IsAlignmentComplete()
    {
        return debounce.calculate(this.m_XController.atSetpoint() && this.m_YController.atSetpoint());
    }

    private void ConfigureDrivetrain()
    {
        this.m_PackLog = new PackLog("Drivetrain");
        this.ConfigureAutobuilder();
        this.ConfigurePIDControllers();
        this.m_SwerveFieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle().withHeadingPID(3, 0.0, 0);
        this.m_SwerveFieldCentricFacingAngle.withRotationalDeadband(DegreesPerSecond.of(0.5));
        this.m_SwerveFieldCentricFacingAngle = m_SwerveFieldCentricFacingAngle.withSteerRequestType(SteerRequestType.Position);
        this.m_SwerveRequestRobot = new SwerveRequest.RobotCentric();
        this.m_SwerveRequestSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        this.m_PackLog.Log("End configuration.");
    }

    private void ConfigurePIDControllers()
    {
        this.m_XController = new PIDController(10, 0.0,0); // 20 0.1 0.025
        this.m_XController.setTolerance(Meters.convertFrom(0.5, Inches), MetersPerSecond.convertFrom(0.5, InchesPerSecond));
        SmartDashboard.putData(this.m_XController);

        this.m_YController = new PIDController (10, 0.0 ,0);
        this.m_YController.setTolerance(Meters.convertFrom(0.5, Inches), MetersPerSecond.convertFrom(0.5, InchesPerSecond));
        SmartDashboard.putData(this.m_YController);
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
        // Ks 0.15033 Kv 0.92447 Ka 0.076706 P 26.116 I D 1.5506

        AutoBuilder.configure(
                () -> this.getState().Pose, // Robot pose supplier
                (pose) -> this.resetPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> this.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.setControl(m_SwerveRequestSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(15, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0474, 0.0, 0.0) // Rotation PID constants  // 5.0 0.0 0.0
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
}
