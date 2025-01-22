package frc.robot.PhotonVision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateToAngle extends Command 
{
    private final CommandSwerveDrivetrain m_Drivetrain;
    private final Rotation2d Angle;

    private final SwerveRequest.FieldCentricFacingAngle rotationRequest = new SwerveRequest.FieldCentricFacingAngle();
    
    public RotateToAngle(CommandSwerveDrivetrain m_drivetrain, Rotation2d angle) {
        this.m_Drivetrain = m_drivetrain;
        this.Angle = angle;
    }

    public void initialize() {
;        rotationRequest.HeadingController.setPID(
            0.5, 
            0.05, 

            0.01
        );

        rotationRequest.HeadingController.setTolerance(0.1);
        rotationRequest.HeadingController.enableContinuousInput(Math.PI, -Math.PI);
    }

    public void execute() {
        rotationRequest
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(Angle);

        m_Drivetrain.setControl(rotationRequest);
        

        if (rotationRequest.HeadingController.atSetpoint()) {
            System.out.println("Aligned with target angle");
        }
    }

    public boolean isFinished() {
        return rotationRequest.HeadingController.atSetpoint();
    }

    public void end(boolean interrupted) {
        return;
    }
}