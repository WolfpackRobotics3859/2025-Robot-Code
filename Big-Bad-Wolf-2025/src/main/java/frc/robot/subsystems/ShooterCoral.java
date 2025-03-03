package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.MotorManager;

public class ShooterCoral extends SubsystemBase
{
    private VoltageOut m_VoltageRequest;
    private StaticBrake m_Brake;
    private TimeOfFlight m_TOF;

    public ShooterCoral()
    {
        MotorManager.AddMotor("SHOOTER CORAL MOTOR", Hardware.SHOOTER_CORAL_MOTOR);
        MotorManager.ApplyConfigs(ShooterConstants.SHOOTER_CORAL_MOTOR_CONFIG, Hardware.SHOOTER_CORAL_MOTOR);
        m_TOF = new TimeOfFlight(Hardware.CORAL_TOF_SENSOR);
        m_VoltageRequest = new VoltageOut(0);
        m_Brake = new StaticBrake();
    }

    public Command IntakeCoralRoutine()
    {
        return new FunctionalCommand(() -> this.SetCoralVoltage(ShooterConstants.CORAL_INTAKE_VOLTAGE),
                                     () -> {}, 
                                     interrupted -> this.BrakeCoral(),
                                     () -> this.CoralDetected(),
                                     this);
    }

    public Command DeployCoralRoutine()
    {
        return new FunctionalCommand(() -> this.SetCoralVoltage(ShooterConstants.CORAL_DEPLOYMENT_VOLTAGE),
                                     () -> {}, 
                                     interrupted -> this.BrakeCoral(),
                                     () -> !this.CoralDetected(),
                                     this);
    }

    private void BrakeCoral()
    {
        MotorManager.ApplyControlRequest(m_Brake, Hardware.SHOOTER_CORAL_MOTOR);
    }

    private void SetCoralVoltage(double voltage)
    {
        MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.SHOOTER_CORAL_MOTOR);
    }

    private boolean CoralDetected()
    {
        return this.m_TOF.getRange() < ShooterConstants.CORAL_TOF_IN_RANGE_THRESHOLD;
    }

    @Override
    public void periodic() 
    {
        // Intentionally Empty
    }
}
