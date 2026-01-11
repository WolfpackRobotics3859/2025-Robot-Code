package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.PackLog;

public class Drivetrain extends CommandSwerveDrivetrain 
{
    private PackLog m_PackLog;
    private PIDController m_XController;
    private PIDController m_YController;
    private PIDController m_RotationController;
    private SwerveRequest.FieldCentric m_SwerveRequest;

    public Drivetrain(SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(constants, modules);    
        this.ConfigureDrivetrain();
    }

    public Command Align(Pose2d desiredPose)
    {
        this.m_XController.reset();
        this.m_YController.reset();

        this.m_XController.setSetpoint(desiredPose.getX());
        this.m_YController.setSetpoint(desiredPose.getY());
        this.m_RotationController.setSetpoint(desiredPose.getRotation().getDegrees());

        return new FunctionalCommand(() -> {}, 
                                     () -> UpdateRequest(), 
                                     interrupted -> {
                                                        m_PackLog.Log("Alignment command finished.");
                                                    }, 
                                     () -> IsAlignmentComplete(), 
                                     this);
    }

    private void UpdateRequest()
    {
        this.m_SwerveRequest.VelocityX = -this.GetXOutput();
        this.m_SwerveRequest.VelocityY = -this.GetYOutput();
        this.m_SwerveRequest.RotationalRate = this.GetRotationOutput();
        this.setControl(m_SwerveRequest);
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
        this.m_PackLog.Log("Beginning configuration.");
        this.m_XController = new PIDController(25, 0.1,0.025);
        this.m_XController.setTolerance(0.01, 0.05);
        this.m_XController.setIntegratorRange(-TunerConstants.MaxSpeed * 0.1,TunerConstants.MaxSpeed * 0.1);
        this.m_XController.setIZone(1);
        SmartDashboard.putData(this.m_XController);

        this.m_YController = new PIDController (25, 0.1 ,0.025);
        this.m_YController.setTolerance(0.01, 0.1);
        this.m_YController.setIntegratorRange(-TunerConstants.MaxSpeed * 0.1,TunerConstants.MaxSpeed * 0.1);
        this.m_YController.setIZone(1);
        SmartDashboard.putData(this.m_YController);

        this.m_RotationController = new PIDController (0.25, 0.5 ,0);
        this.m_RotationController.setTolerance(1, 1);
        this.m_RotationController.setIntegratorRange(-TunerConstants.MaxAngularRate * 0.1,TunerConstants.MaxAngularRate * 0.1);
        this.m_RotationController.setIZone(10);
        this.m_RotationController.enableContinuousInput(-180, 180);
        SmartDashboard.putData(this.m_RotationController);

        this.m_SwerveRequest = new SwerveRequest.FieldCentric();

        this.m_PackLog.Log("End configuration.");
    }

}
