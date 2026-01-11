package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.MotorManager;

public class ShooterAlgae extends SubsystemBase
{
    private VoltageOut m_VoltageRequest;

    public ShooterAlgae()
    {
        MotorManager.AddMotor("SHOOTER ALGAE MOTOR", Hardware.SHOOTER_ALGAE_MOTOR);
        MotorManager.ApplyConfigs(ShooterConstants.SHOOTER_ALGAE_MOTOR_CONFIG, Hardware.SHOOTER_ALGAE_MOTOR);
        m_VoltageRequest = new VoltageOut(0);
    }

    public Command BeginSweepAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_SWEEPING_VOLTAGE));
    }

    public Command HoldAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_HOLDING_VOLTAGE));
    }

    public Command StopAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(0));
    }

    public Command DeployAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_PROCESSOR_DEPLOYMENT_VOLTAGE));
    }

    public void SetAlgaeVoltage(double voltage)
    {
        MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.SHOOTER_ALGAE_MOTOR);
    }

    @Override
    public void periodic() 
    {
        // Intentionally Empty
    }
}
