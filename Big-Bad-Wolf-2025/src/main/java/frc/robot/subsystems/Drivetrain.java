package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;

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
    SwerveRequest.SwerveDriveBrake m_BrakeRequest;

    private final SwerveRequest.FieldCentric m_OperatorDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); 

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

    public Command Align(Pose2d goalPose)
    {
        return new FunctionalCommand(() -> 
                                        {
                                            this.m_XController.reset();
                                            this.m_YController.reset();
                                            this.m_XController.setSetpoint(goalPose.getX());
                                            this.m_YController.setSetpoint(goalPose.getY());
                                            
                                            if (alliance == Alliance.Blue)
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation();
                                            }
                                            else
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation().rotateBy(Rotation2d.k180deg);
                                            }
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> IsAlignmentComplete(), 
                                     this);
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

                                            if (alliance == Alliance.Blue)
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation();
                                            }
                                            else
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation().rotateBy(Rotation2d.k180deg);
                                            }
                                            
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
                                            
                                            if (alliance == Alliance.Blue)
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation();
                                            }
                                            else
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation().rotateBy(Rotation2d.k180deg);
                                            }
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {}, 
                                     () -> false, 
                                     this);
    }

    public Command Brake()
    {
        return this.runOnce(() -> this.applyRequest(() -> this.m_BrakeRequest));
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
                                            
                                            if (alliance == Alliance.Blue)
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation();
                                            }
                                            else
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation().rotateBy(Rotation2d.k180deg);
                                            }
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {}, 
                                     () -> this.IsAlignmentComplete(), 
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
                                            
                                            if (alliance == Alliance.Blue)
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation();
                                            }
                                            else
                                            {
                                                this.m_SwerveFieldCentricFacingAngle.TargetDirection = goalPose.getRotation().rotateBy(Rotation2d.k180deg);
                                            }
                                        }, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                        this.applyRequest(() -> this.m_BrakeRequest);
                                                    }, 
                                     () -> false, 
                                     this);
    }

    public Command RotationTesting(Pose2d testPose)
    {
        return new FunctionalCommand(() -> 
                                        {
                                            this.m_SwerveFieldCentricFacingAngle.TargetDirection = Rotation2d.kZero;
                                        }, 
                                     () -> UpdateRotationOnly(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> false, 
                                     this);
    }

    public Command AlignXTesting(Pose2d testPose)
    {
        return new FunctionalCommand(() -> 
                                        {
                                            this.m_XController.reset();
                                            this.m_XController.setSetpoint(testPose.getX());
                                            this.m_SwerveFieldCentricFacingAngle.TargetDirection = Rotation2d.kZero;
                                        }, 
                                     () -> UpdateXOnly(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> this.m_XController.atSetpoint(), 
                                     this);
    }

    public Command AlignYTesting(Pose2d testPose)
    {
        return new FunctionalCommand(() -> 
                                        {
                                            this.m_YController.reset();
                                            this.m_YController.setSetpoint(testPose.getY());
                                            this.m_SwerveFieldCentricFacingAngle.TargetDirection = Rotation2d.kZero;
                                        }, 
                                     () -> UpdateYOnly(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> this.m_YController.atSetpoint(), 
                                     this);
    }

    private Pose2d GetGoalCoralPose()
    {
        if(alliance == Alliance.Blue)
        {
            if(DataStuff.GetSide().getValue() == 0)
            {
                return DrivetrainConstants.BLUE_LEFT_ALIGNMENTS[DataStuff.GetFace().getValue()];
            }
            return DrivetrainConstants.BLUE_RIGHT_ALIGNMENTS[DataStuff.GetFace().getValue()];
        }

        if(DataStuff.GetSide().getValue() == 0)
        {
            return DrivetrainConstants.RED_LEFT_ALIGNMENTS[DataStuff.GetFace().getValue()];
        }
        return DrivetrainConstants.RED_RIGHT_ALIGNMENTS[DataStuff.GetFace().getValue()];
    }

    private Pose2d GetGoalCleanPose()
    {
        if(alliance == Alliance.Blue)
        {
            return DrivetrainConstants.BLUE_CENTER_ALIGNMENTS[DataStuff.GetFace().getValue()];
        }
        return DrivetrainConstants.RED_CENTER_ALIGNMENTS[DataStuff.GetFace().getValue()];
    }

    private Pose2d GetGoalStagingPose()
    {
        if(alliance == Alliance.Blue)
        {
            return DrivetrainConstants.BLUE_STAGING_POSES[DataStuff.GetFace().getValue()];
        }
        return DrivetrainConstants.RED_STAGING_POSES[DataStuff.GetFace().getValue()];
    }

    private Pose2d GetGoalStagingPoseUniversal()
    {
            return DrivetrainConstants.BLUE_STAGING_POSES[DataStuff.GetFace().getValue()];
    }

    public Command PathfindToStaging(double velocity, double acceleration, double endVelocity)
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            velocity, acceleration,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            this.GetGoalStagingPose(),
            constraints,
            endVelocity // Goal end velocity in meters/sec
        );
    }

    public Command PathfindToStagingUniversal(double velocity, double acceleration, double endVelocity)
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            velocity, acceleration,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        if(this.alliance == Alliance.Red)
        {
            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            return AutoBuilder.pathfindToPoseFlipped(
                this.GetGoalStagingPoseUniversal(),
                constraints,
                endVelocity // Goal end velocity in meters/sec
        );
        }

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            this.GetGoalStagingPoseUniversal(),
            constraints,
            endVelocity // Goal end velocity in meters/sec
        );
    }

    public Command PathfindToCoral(double velocity, double acceleration, double endVelocity)
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            velocity, acceleration,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            this.GetGoalCoralPose(),
            constraints,
            endVelocity // Goal end velocity in meters/sec
        );
    }

    public Command PathfindToPose(Pose2d goalPose, double velocity, double acceleration, double endVelocity)
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            velocity, acceleration,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            goalPose,
            constraints,
            endVelocity // Goal end velocity in meters/sec
        );
    }

    public Command PathfindThenFollowPath(PathPlannerPath path, double approachVelocity, double approachAcceleration)
    {
        PathConstraints constraints = new PathConstraints(
        approachVelocity, approachAcceleration,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    public Command DefaultDrive(Supplier<Double> thrust, Supplier<Double> strafe, Supplier<Double> rotation)
    {
        return this.applyRequest(() -> this.m_OperatorDriveRequest.withVelocityX(-thrust.get() * TunerConstants.MaxSpeed * 0.8)
                                                                  .withVelocityY(-strafe.get() * TunerConstants.MaxSpeed * 0.8)
                                                                  .withRotationalRate(-rotation.get() * TunerConstants.MaxAngularRate));
    }

    public Command RearDriveSnap(Supplier<Double> thrust, Supplier<Double> strafe)
    {
        return this.run(() -> {
            Rotation2d goalRotation;
            if(this.alliance == Alliance.Blue)
            {
                goalRotation = Rotation2d.k180deg;
            }
            else
            {
                goalRotation = Rotation2d.kZero;
            }
            this.setControl(this.m_SwerveFieldCentricFacingAngle.withVelocityX(-thrust.get() * TunerConstants.MaxSpeed * 0.8)
                                                                .withVelocityY(-strafe.get() * TunerConstants.MaxSpeed * 0.8)
                                                                .withTargetDirection(goalRotation));

        });
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

    private void UpdateYOnly()
    {
        if(alliance == Alliance.Red)
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = 0;
            this.m_SwerveFieldCentricFacingAngle.VelocityY = -this.GetYOutput();
            this.setControl(m_SwerveFieldCentricFacingAngle);
        }
        else
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = 0;
            this.m_SwerveFieldCentricFacingAngle.VelocityY = this.GetYOutput();
            this.setControl(m_SwerveFieldCentricFacingAngle);
        }
    }

    private void UpdateXOnly()
    {
        if(alliance == Alliance.Red)
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = -this.GetXOutput();
            this.m_SwerveFieldCentricFacingAngle.VelocityY = 0;
            this.setControl(m_SwerveFieldCentricFacingAngle);
        }
        else
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = this.GetXOutput();
            this.m_SwerveFieldCentricFacingAngle.VelocityY = 0;
            this.setControl(m_SwerveFieldCentricFacingAngle);
        }
    }

    private void UpdateRotationOnly()
    {
        if(alliance == Alliance.Red)
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = 0;
            this.m_SwerveFieldCentricFacingAngle.VelocityY = 0;
            this.setControl(m_SwerveFieldCentricFacingAngle);
        }
        else
        {
            this.m_SwerveFieldCentricFacingAngle.VelocityX = 0;
            this.m_SwerveFieldCentricFacingAngle.VelocityY = 0;
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

    Debouncer debounce = new Debouncer(0.00, DebounceType.kRising);

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
        this.m_BrakeRequest = new SwerveRequest.SwerveDriveBrake();

        this.m_PackLog.Log("End configuration.");
    }

    private void ConfigurePIDControllers()
    {
        this.m_XController = new PIDController(5, 10,0.7); // 5 10 0.7
        this.m_XController.setTolerance(Meters.convertFrom(0.5, Inches), MetersPerSecond.convertFrom(3, InchesPerSecond));
        this.m_XController.setIZone(0.1); // 0.0762
        SmartDashboard.putData(this.m_XController);

        this.m_YController = new PIDController (5, 10,0.7);
        this.m_YController.setTolerance(Meters.convertFrom(0.5, Inches), MetersPerSecond.convertFrom(3, InchesPerSecond));
        this.m_XController.setIZone(0.1);
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

        AutoBuilder.configure(
                () -> this.getState().Pose, // Robot pose supplier
                (pose) -> this.resetPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> this.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.setControl(m_SwerveRequestSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(15, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants  // 5.0 0.0 0.0  // 1.074
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
