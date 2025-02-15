// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final TalonFX m_ElevatorMotorLeft;
  private final TalonFX m_ElevatorMotorRight;
  private final CANdi m_CANdi;

  private final VoltageOut m_VoltageRequest;
  private final MotionMagicVoltage m_PositionRequest;
  private final StaticBrake m_BrakeRequest;

  private final SendableChooser<LEVELS> m_LevelChooser;
  
  private SysIdRoutine m_SysIdRoutine;

  /**
   * Constructor which runs anything in it upon initialization and creates a new object.
   */
  public Elevator()
  {
    MotorManager.AddMotor("ELEVATOR LEFT MOTOR", Hardware.ELEVATOR_MOTOR_LEFT);
    MotorManager.AddMotor("ELEVATOR RIGHT MOTOR", Hardware.ELEVATOR_MOTOR_RIGHT);

    m_ElevatorMotorLeft = MotorManager.GetMotor(Hardware.ELEVATOR_MOTOR_LEFT);
    m_ElevatorMotorRight = MotorManager.GetMotor(Hardware.ELEVATOR_MOTOR_RIGHT);
    m_CANdi = new CANdi(Hardware.CANDI_0);

    MotorManager.ApplyConfigs(ElevatorConstants.LEFT_MOTOR_CONFIG, Hardware.ELEVATOR_MOTOR_LEFT);
    MotorManager.ApplyConfigs(ElevatorConstants.RIGHT_MOTOR_CONFIG, Hardware.ELEVATOR_MOTOR_RIGHT);

    MotorManager.ApplyControlRequest(new Follower(Hardware.ELEVATOR_MOTOR_LEFT, false), Hardware.ELEVATOR_MOTOR_RIGHT);

    m_VoltageRequest = new VoltageOut(0);
    m_PositionRequest = new MotionMagicVoltage(0);
    m_BrakeRequest = new StaticBrake();

    m_LevelChooser = new SendableChooser<>();
    m_LevelChooser.setDefaultOption("Home", LEVELS.HOME);
    m_LevelChooser.addOption("ONE", LEVELS.ONE);
    m_LevelChooser.addOption("TWO", LEVELS.TWO);
    m_LevelChooser.addOption("THREE", LEVELS.THREE);
    m_LevelChooser.addOption("FOUR", LEVELS.FOUR);
    SmartDashboard.putData("Level Chooser", m_LevelChooser);
  }

  public Command MoveToSmartdashboardSelectedLevel()
  {
    return this.MoveToLevel(m_LevelChooser.getSelected());
  }

  public Command MoveToLevel(LEVELS level)
  {
    return this.runOnce(() -> this.SetPosition(level.getValue()));
  }

  public Command ZeroElevator()
  {
    return new FunctionalCommand(
      // Begin moving the intake to a desired position.
      () -> this.SetVoltage(ElevatorConstants.HOMING_VOLTAGE),
      () -> {},
      interrupted -> this.BrakeElevator(),
      // Ends the command once the intake is in the desired position.
      () -> m_ElevatorMotorLeft.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround,
      this
    );
  }

  public Command ApplyVoltage(double voltage)
  {
    return this.runOnce(() -> this.SetVoltage(voltage));
  }

  private Elevator SetVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.ELEVATOR_MOTOR_LEFT);
    return this;
  }

  private Elevator SetPosition(double position)
  {
    MotorManager.ApplyControlRequest(m_PositionRequest.withPosition(position), Hardware.ELEVATOR_MOTOR_LEFT);
    return this;
  }

  private Elevator BrakeElevator()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.ELEVATOR_MOTOR_LEFT);
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
         Volts.of(0.5).per(Seconds),         // Use default ramp rate (1 V/s)
         Volts.of(0.5), // Reduce dynamic step voltage to 4 to prevent brownout
         null,          // Use default timeout (10 s)
         (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_ElevatorMotorLeft.setControl(new VoltageOut(volts.in(Volts))),
         null,
         this
      )
   );
   return this.m_SysIdRoutine;
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    // Intentionally Empty
  }
}
