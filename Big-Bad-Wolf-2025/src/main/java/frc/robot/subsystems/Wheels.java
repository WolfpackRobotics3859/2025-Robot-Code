package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Hardware;
import frc.robot.constants.WheelConstants;
import frc.robot.constants.WheelConstants.VoltageSpeeds;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Lights.LIGHT_CODES;
import frc.robot.utilities.MotorManager;
import frc.robot.utilities.PackLog;

public class Wheels extends SubsystemBase
{
    private PackLog m_PackLog;

    private Lights m_Lights;

    private TalonFX m_WheelsMotor;
    private TimeOfFlight m_ForwardTOF;
    private TimeOfFlight m_RearTOF;

    private VoltageOut m_VoltageRequest;
    private PositionVoltage m_PositionRequest;

    private StatusSignal<AngularVelocity> m_VelocitySignal;
    private StatusSignal<Angle> m_PositionSignal;

    private boolean m_SafetySignal;
    private Debouncer m_SafetySignalDebouncer;

    private int m_CenteringState;
    

    public Wheels()
    {
        this.m_PackLog = new PackLog(WheelConstants.NAME);

        MotorManager.AddMotor("WHEEL MOTOR", Hardware.CORAL_MOTOR_ID);
        MotorManager.ApplyConfigs(WheelConstants.MOTOR_CONFIG, Hardware.CORAL_MOTOR_ID);
        this.m_WheelsMotor = MotorManager.GetMotor(Hardware.CORAL_MOTOR_ID);

        this.m_VelocitySignal = this.m_WheelsMotor.getVelocity();
        this.m_PositionSignal = this.m_WheelsMotor.getPosition();

        m_ForwardTOF = new TimeOfFlight(Hardware.CORAL_FORWARD_TOF_SENSOR);
        m_RearTOF = new TimeOfFlight(Hardware.CORAL_REAR_TOF_SENSOR);

        m_VoltageRequest = new VoltageOut(0);
        m_PositionRequest = new PositionVoltage(0);

        this.m_SafetySignal = false;
        this.m_SafetySignalDebouncer = new Debouncer(WheelConstants.SAFETY_SIGNAL_DEBOUNCE_SECONDS);

        this.m_CenteringState = 0;
    }

    @Override
    public void periodic() 
    {
        // Intentionally Empty
    }

    public Command BeginCoralIntakeRoutine()
    {
        return new FunctionalCommand(() -> this.SetVoltage(VoltageSpeeds.INTAKE),
                                     () -> {},
                                     interrupted -> {}, 
                                     () -> this.ForwardActive(),
                                     this);
    }

    public Command CenterCoral()
    {
        Command returnCommand = new FunctionalCommand(() -> this.ResetCentering(),
                                                      () -> {},
                                                      interrupted -> 
                                                      {
                                                        this.SetPosition(this.m_PositionSignal.refresh().getValueAsDouble());
                                                        m_Lights.LightChooser(LIGHT_CODES.SOLID_BLUE);
                                                      }, 
                                                      () -> this.SimpleCentering(),
                                                      this);
        return returnCommand.withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withTimeout(5);
    }

    public Command IntakeAlgae()
    {
        return new FunctionalCommand(() -> this.SetVoltage(VoltageSpeeds.SWEEP),
                                     () -> {},
                                     interrupted -> 
                                     {
                                        this.SetPosition(this.m_PositionSignal.refresh().getValueAsDouble());
                                     }, 
                                     () -> this.SenseAlgae() || this.AnyActive(),
                                     this);
    }

    public Command DeployAlgae()
    {
        return new FunctionalCommand(() -> this.SetVoltage(VoltageSpeeds.BARGE),
                                     () -> {},
                                     interrupted -> 
                                     {
                                        this.SetVoltage(VoltageSpeeds.ZERO);
                                     }, 
                                     () -> this.AnyActive(),
                                     this);
    }


    /**
     * The safety signal is pulled high when other mechanisms may move safely.
     * @return provides the safety signal
     */
    public BooleanSupplier GetSafetySignal()
    {
        return () -> this.m_SafetySignal;
    }

    private void SetVoltage(VoltageSpeeds speed)
    {
        MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(speed.getValue()), Hardware.CORAL_MOTOR_ID);
    }

    private void SetPosition(double position)
    {
        MotorManager.ApplyControlRequest(m_PositionRequest.withPosition(position), Hardware.CORAL_MOTOR_ID);
    }

    private void UpdateSafetySignal()
    {
        this.m_SafetySignal = this.m_SafetySignalDebouncer.calculate(this.RearActive());
    }

    private boolean ForwardActive()
    {
        return this.m_ForwardTOF.getRange() < WheelConstants.TOF_IN_RANGE_THRESHOLD;
    }

    private boolean RearActive()
    {
        return this.m_RearTOF.getRange() < WheelConstants.TOF_IN_RANGE_THRESHOLD;
    }

    private boolean AnyActive()
    {
        return this.ForwardActive() || RearActive();
    }

    private boolean SenseAlgae()
    {
        this.m_VelocitySignal.refresh();
        return Math.abs(this.m_VelocitySignal.getValueAsDouble()) < WheelConstants.ALGAE_RESISTANCE_VELOCITY_THRESHOLD;
    }

    private void ResetCentering()
    {
        this.m_CenteringState = 0;
    }

    private boolean SimpleCentering()
    {
        switch(this.m_CenteringState)
        {
            case 0:
                if(!this.AnyActive())
                {
                    return true;
                }

                if(this.RearActive())
                {
                    this.SetVoltage(VoltageSpeeds.INTAKE);
                    this.m_CenteringState = 3;
                }
                else
                {
                    this.SetVoltage(VoltageSpeeds.REVERSE_CENTERING);
                    this.m_CenteringState = 1;
                }
            break;

            case 1:
                if(this.RearActive())
                {
                    this.SetVoltage(VoltageSpeeds.FORWARD_CENTERING);
                    this.m_CenteringState = 2;
                }
            break;

            case 2:
                if(!this.RearActive())
                {
                    this.SetVoltage(VoltageSpeeds.ZERO);
                    return true;
                }
            break;

            case 3:
                if(!this.RearActive())
                {
                    this.m_CenteringState = 0;
                }
            break;

            default:
                this.SetVoltage(VoltageSpeeds.ZERO);
                this.m_PackLog.Log("Unexpected centering state encountered.");
            break;
        }
        return false;
    }
}
