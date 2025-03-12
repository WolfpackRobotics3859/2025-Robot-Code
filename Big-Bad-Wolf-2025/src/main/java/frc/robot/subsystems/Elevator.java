
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.constants.Hardware;
import frc.robot.utilities.MotorManager;

public class Elevator extends SubsystemBase
{
  // Hardware
  private final TalonFX m_ElevatorMotor;
  private final CANdi m_CANdi;

  private final VoltageOut m_VoltageRequest;
  private final MotionMagicVoltage m_PositionRequest;
  private final StaticBrake m_BrakeRequest;
  private final CoastOut m_CoastRequest;

  private final StatusSignal<AngularVelocity> m_ElevatorRPS;
  private final StatusSignal<Angle> m_ElevatorPosition;
  
  private SysIdRoutine m_SysIdRoutine;

  /**
   * Constructor which runs anything in it upon initialization and creates a new object.
   */
  public Elevator()
  {
    MotorManager.AddMotor("ELEVATOR LEFT MOTOR", Hardware.ELEVATOR_MOTOR);

    m_ElevatorMotor = MotorManager.GetMotor(Hardware.ELEVATOR_MOTOR);
    MotorManager.ApplyConfigs(ElevatorConstants.ELEVATOR_MOTOR_CONFIG, Hardware.ELEVATOR_MOTOR);

    this.m_ElevatorRPS = this.m_ElevatorMotor.getVelocity();
    this.m_ElevatorPosition = this.m_ElevatorMotor.getPosition();

    m_CANdi = new CANdi(Hardware.CANDI_0);

    m_VoltageRequest = new VoltageOut(0);
    m_PositionRequest = new MotionMagicVoltage(0);
    m_BrakeRequest = new StaticBrake();
    m_CoastRequest = new CoastOut();
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    if(m_CANdi.isConnected() && (m_CANdi.getS1State().getValue() == S1StateValue.Low) && m_CANdi.getS1Closed().refresh().getValue())
    {
      this.m_ElevatorMotor.setPosition(0);
    }
    SmartDashboard.putNumber("Elevator Position :)", this.m_ElevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Closed Loop Error", m_ElevatorMotor.getClosedLoopError().getValueAsDouble());
    if(m_CANdi.isConnected() && (m_CANdi.getS1State().getValue() == S1StateValue.Low) && m_CANdi.getS1Closed().refresh().getValue())
    {
      this.m_ElevatorMotor.setPosition(0);
    }
    SmartDashboard.putNumber("Elevator Position :)", this.m_ElevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Closed Loop Error", m_ElevatorMotor.getClosedLoopError().getValueAsDouble());
  }

  public Command MoveToLevel(LEVELS level)
  {
    return new FunctionalCommand(
      () -> this.ApplyPosition(level.getValue()),
      () -> {},
      interrupted -> {},
      () -> isInPosition(0.05),
      this
    );
  }

  public Command ZeroElevator()
  {
    return new FunctionalCommand(
      () -> this.SetVoltage(ElevatorConstants.HOMING_VOLTAGE),
      () -> {},
      interrupted -> this.ApplyPosition(0),
      () -> m_CANdi.isConnected() && (m_CANdi.getS1State().getValue() == S1StateValue.Low) && m_CANdi.getS1Closed().getValue(),
      this
    );
  }

  public Command SetVoltage(double voltage)
  {
    return this.runOnce(() -> this.ApplyVoltage(voltage));
  }

  public Command SetBrake()
  {
    return this.runOnce(() -> this.ApplyBrake());
  }

  public Command SetCoast()
  {
    return this.runOnce(() -> this.ApplyCoast());
  }

  private Elevator ApplyVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.ELEVATOR_MOTOR);
    return this;
  }

  private Elevator ApplyPosition(double position)
  {
    MotorManager.ApplyControlRequest(m_PositionRequest.withPosition(position), Hardware.ELEVATOR_MOTOR);
    return this;
  }

  private Elevator ApplyBrake()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.ELEVATOR_MOTOR);
    return this;
  }

  private Elevator ApplyCoast()
  {
    MotorManager.ApplyControlRequest(m_CoastRequest, Hardware.ELEVATOR_MOTOR);
    return this;
  }

  public SysIdRoutine getSysIdRoutine()
  {
    return this.m_SysIdRoutine;
  }

  // To-do: Move sysId settings to the constants file
  public SysIdRoutine BuildSysIdRoutine()
  {
    this.m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(0.5).per(Seconds),  // Ramp Rate in Volts / Seconds
         Volts.of(2), // Dynamic Step Voltage
         null,          // Use default timeout (10 s)
         (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_ElevatorMotor.setControl(new VoltageOut(volts.in(Volts))),
         null,
         this
      )
   );
   return this.m_SysIdRoutine;
  }

  private boolean isInPosition(double tolerance)
  {
    return Math.abs(this.m_ElevatorMotor.getPosition().getValueAsDouble() - this.m_PositionRequest.Position) < tolerance;
  }

  private boolean newIsInPosition(double positionTolerance, double derivativeTolerance)
  {
    StatusSignal.refreshAll(this.m_ElevatorRPS, this.m_ElevatorPosition);
    if((Math.abs(this.m_ElevatorRPS.getValueAsDouble()) > derivativeTolerance) || (Math.abs(this.m_ElevatorPosition.getValueAsDouble()) > positionTolerance))
    {
      return false;
    }
    return true;
  }

  private void BuildToolbox()
  {
    SmartDashboard.putData("Static Brake Elevator", this.SetBrake().ignoringDisable(true));
    SmartDashboard.putData("Coast Elevator", this.SetCoast().ignoringDisable(true));
    SmartDashboard.putData("Zero Elevator (Hall Effect)", this.ZeroElevator());
  }

}