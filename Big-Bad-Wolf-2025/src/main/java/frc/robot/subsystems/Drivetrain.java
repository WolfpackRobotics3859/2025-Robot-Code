package frc.robot.subsystems;

import java.util.function.Supplier;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.CameraManager;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.PackLog;

public class Drivetrain extends CommandSwerveDrivetrain 
{
    private PackLog m_PackLog;

    private CameraManager m_CameraManager;

    private PIDController m_XController;
    private PIDController m_YController;
    private PIDController m_RotationController;

    private SwerveRequest.FieldCentric m_SwerveRequestField;
    private SwerveRequest.RobotCentric m_SwerveRequestRobot;
    private SwerveRequest.ApplyRobotSpeeds m_SwerveRequestSpeeds;

    private final SwerveRequest.FieldCentric m_OperatorDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public Drivetrain(SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(constants, modules);    
        this.ConfigureDrivetrain();
    }

    @Override
    public void periodic() 
    {
        super.periodic();
        this.m_CameraManager.UpdateCameras(this::addVisionMeasurement);
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

    public Command AlignCenter()
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
        return this.applyRequest(() -> this.m_OperatorDriveRequest.withVelocityX(-thrust.get() * TunerConstants.MaxSpeed * 0.8)
                                                                  .withVelocityY(-strafe.get() * TunerConstants.MaxSpeed * 0.8)
                                                                  .withRotationalRate(-rotation.get() * TunerConstants.MaxAngularRate));
    }

    private void UpdateRequest()
    {
        this.m_SwerveRequestField.VelocityX = -this.GetXOutput();
        this.m_SwerveRequestField.VelocityY = -this.GetYOutput();
        this.m_SwerveRequestField.RotationalRate = this.GetRotationOutput();
        this.setControl(m_SwerveRequestField);
    }

    private double GetXOutput()
    {
        return MathUtil.clamp(this.m_XController.calculate(this.getState().Pose.getX()), -TunerConstants.MaxSpeed, TunerConstants.MaxSpeed);
    }

    private double GetYOutput()
    {
        return MathUtil.clamp(this.m_YController.calculate(this.getState().Pose.getY()), -TunerConstants.MaxSpeed, TunerConstants.MaxSpeed);
    }

    private double GetRotationOutput()
    {
        return MathUtil.clamp(this.m_RotationController.calculate(this.getState().Pose.getRotation().getDegrees()), -TunerConstants.MaxAngularRate, TunerConstants.MaxAngularRate);
    }

    private boolean IsAlignmentComplete()
    {
        return this.m_XController.atSetpoint() && this.m_YController.atSetpoint();
    }

    private void ConfigureDrivetrain()
    {
        this.m_PackLog = new PackLog("Drivetrain");
        this.ConfigureAutobuilder();
        this.ConfigurePIDControllers();

        this.m_CameraManager = new CameraManager();
        this.m_CameraManager.InitializeCameras(CameraConstants.CAMERA_PIPELINES, CameraConstants.ROBOT_TO_CAM_TRANFORMS);

        this.m_SwerveRequestField = new SwerveRequest.FieldCentric();
        this.m_SwerveRequestRobot = new SwerveRequest.RobotCentric();
        this.m_SwerveRequestSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        this.m_PackLog.Log("End configuration.");
    }

    private void ConfigurePIDControllers()
    {
        this.m_PackLog.Log("Beginning configuration.");
        this.m_XController = new PIDController(25, 0.1,0.025);
        this.m_XController.setTolerance(0.01, 0.025);
        this.m_XController.setIntegratorRange(-TunerConstants.MaxSpeed * 0.1,TunerConstants.MaxSpeed * 0.1);
        this.m_XController.setIZone(1);
        SmartDashboard.putData(this.m_XController);

        this.m_YController = new PIDController (25, 0.1 ,0.025);
        this.m_YController.setTolerance(0.01, 0.025);
        this.m_YController.setIntegratorRange(-TunerConstants.MaxSpeed * 0.1,TunerConstants.MaxSpeed * 0.1);
        this.m_YController.setIZone(1);
        SmartDashboard.putData(this.m_YController);

        this.m_RotationController = new PIDController (0.25, 0.5 ,0);
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
}
