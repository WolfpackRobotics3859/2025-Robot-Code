package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.WheelConstants;
import frc.robot.constants.WristConstants;
import frc.robot.utilities.MotorManager;
import frc.robot.utilities.PackLog;

public class Wheels extends SubsystemBase
{
    private PackLog m_PackLog;

    private TimeOfFlight m_ForwardTOF;
    private TimeOfFlight m_RearToF;

    private VoltageOut m_VoltageRequest;
    private StaticBrake m_Brake;
    

    public Wheels()
    {
        this.m_PackLog = new PackLog(WheelConstants.NAME);

        MotorManager.AddMotor("WHEEL MOTOR", Hardware.CORAL_MOTOR_ID);
        MotorManager.ApplyConfigs(WheelConstants.MOTOR_CONFIG, Hardware.CORAL_MOTOR_ID);

        m_ForwardTOF = new TimeOfFlight(Hardware.CORAL_FORWARD_TOF_SENSOR);
        m_RearToF = new TimeOfFlight(Hardware.CORAL_REAR_TOF_SENSOR);

        m_VoltageRequest = new VoltageOut(0);
        m_Brake = new StaticBrake();
    }

    public Command IntakeCoralRoutine()
    {
        return new FunctionalCommand(() -> this.SetCoralVoltage(WristConstants.CORAL_INTAKE_VOLTAGE),
                                     () -> {}, 
                                     interrupted -> this.BrakeCoral(),
                                     () -> this.CoralDetected(),
                                     this);
    }

    public Command DeployCoralRoutine()
    {
        return new FunctionalCommand(() -> this.SetCoralVoltage(WristConstants.CORAL_DEPLOYMENT_VOLTAGE),
                                     () -> {}, 
                                     interrupted -> this.BrakeCoral(),
                                     () -> !this.CoralDetected(),
                                     this);
    }

    public Command StopCoral()
    {
        return this.runOnce(() -> BrakeCoral());
    }

    private void BrakeCoral()
    {
        MotorManager.ApplyControlRequest(m_Brake, Hardware.CORAL_MOTOR_ID);
    }

    private void SetCoralVoltage(double voltage)
    {
        MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CORAL_MOTOR_ID);
    }

    private boolean CoralDetected()
    {
        return this.m_TOF.getRange() < WheelConstants.TOF_IN_RANGE_THRESHOLD;
    }

    @Override
    public void periodic() 
    {
        // Intentionally Empty
    }
}
