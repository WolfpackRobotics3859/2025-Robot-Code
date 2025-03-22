package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.MotorManager;

public class ShooterAlgae extends SubsystemBase
{

    // double motorCurrent;
    // double stallCurrentThreshold;

    private TalonFX m_AlgaeMotor;
    private VoltageOut m_VoltageRequest;
    private MotionMagicVoltage m_PositionRequest;

    private double holdPosition;
    private boolean applyHoldingPosition;

    public ShooterAlgae()
    {
        MotorManager.AddMotor("SHOOTER ALGAE MOTOR", Hardware.SHOOTER_ALGAE_MOTOR);
        MotorManager.ApplyConfigs(ShooterConstants.SHOOTER_ALGAE_MOTOR_CONFIG, Hardware.SHOOTER_ALGAE_MOTOR);
        m_AlgaeMotor = MotorManager.GetMotor(Hardware.SHOOTER_ALGAE_MOTOR);
        m_VoltageRequest = new VoltageOut(0);
    }

    private double updateHoldingposition()
    {
        return m_AlgaeMotor.getPosition().getValueAsDouble();
    }

    public Command CleanAlgaeRoutine()
    {
        return new FunctionalCommand(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_SWEEPING_VOLTAGE),
                                     () -> holdPosition = this.updateHoldingposition(),
                                     interrupted -> SetAlgaeVoltage(ShooterConstants.ALGAE_HOLDING_VOLTAGE),   
                                     ()-> false, 
                                     this);
    }

    public Command DeployAlgaeRoutine()
    {
        return new FunctionalCommand(() -> 
                                     {
                                        this.applyHoldingPosition = false;
                                        this.SetAlgaeVoltage(ShooterConstants.ALGAE_BARGE_SHOOTING_VOLTAGE);
                                     },
                                     () -> {}, 
                                     interrupted -> this.SetAlgaeVoltage(0),
                                     ()-> false,
                                     this);
    }

    public Command BeginCleanAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_SWEEPING_VOLTAGE));
    }

    public Command StopAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(0));
    }

    public Command HoldAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_HOLDING_VOLTAGE));
    }


    public Command ProcessAlgae()
    {
        return this.runOnce(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_PROCESSOR_DEPLOYMENT_VOLTAGE));
    }

    public void SetAlgaeVoltage(double voltage)
    {
        MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.SHOOTER_ALGAE_MOTOR);
    }

    private ShooterAlgae SetAlgaePosition(double position)
    {
        MotorManager.ApplyControlRequest(m_PositionRequest.withPosition(position), Hardware.SHOOTER_ALGAE_MOTOR);
        return this;
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Roller Position lol", MotorManager.GetMotor(Hardware.SHOOTER_ALGAE_MOTOR).getPosition().getValueAsDouble());
    }
}
