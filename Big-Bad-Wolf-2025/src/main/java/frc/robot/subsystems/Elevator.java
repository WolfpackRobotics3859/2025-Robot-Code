// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.HEIGHTS;
import frc.robot.lib.Positions.Level;
import frc.robot.constants.Hardware;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.MotorManager;
import frc.robot.utilities.PackLog;

public class Elevator extends SubsystemBase
{
  private final PackLog m_Logger;

  private final TalonFX m_ElevatorMotor;

  private final VoltageOut m_VoltageRequest;
  private final MotionMagicVoltage m_PositionRequest;
  private final StaticBrake m_BrakeRequest;
  private final CoastOut m_CoastRequest;

  private final StatusSignal<AngularVelocity> m_ElevatorRPS;
  private final StatusSignal<Angle> m_ElevatorPosition;

  private final NetworkTable m_Table;

  private final DoublePublisher m_PositionPub;
  private final DoublePublisher m_VelocityPub;
  private final DoublePublisher m_GoalPositionPub;
  private final StringPublisher m_SelectedHeightPub;
  private final BooleanPublisher m_InPositionPub;
  
  private SysIdRoutine m_SysIdRoutine;

  public Elevator()
  {
    this.m_Logger = new PackLog(ElevatorConstants.NAME);

    MotorManager.AddMotor("ELEVATOR MOTOR", Hardware.ELEVATOR_MOTOR);

    m_ElevatorMotor = MotorManager.GetMotor(Hardware.ELEVATOR_MOTOR);
    MotorManager.ApplyConfigs(ElevatorConstants.ELEVATOR_MOTOR_CONFIG, Hardware.ELEVATOR_MOTOR);

    this.m_ElevatorRPS = this.m_ElevatorMotor.getVelocity();
    this.m_ElevatorPosition = this.m_ElevatorMotor.getPosition();

    m_VoltageRequest = new VoltageOut(0);
    m_PositionRequest = new MotionMagicVoltage(0);
    m_BrakeRequest = new StaticBrake();
    m_CoastRequest = new CoastOut();

    this.m_Table = NetworkTableInstance.getDefault().getTable(ElevatorConstants.NAME);
    this.m_PositionPub = m_Table.getDoubleTopic("Position").publish();
    this.m_VelocityPub = m_Table.getDoubleTopic("Velocity").publish();
    this.m_GoalPositionPub = m_Table.getDoubleTopic("Goal Position").publish();
    this.m_SelectedHeightPub = m_Table.getStringTopic("Selected Height").publish();
    this.m_InPositionPub = m_Table.getBooleanTopic("In Position").publish();

    this.BuildToolbox();
  }

  @Override
  public void periodic()
  {
    this.UpdateTelemetry();
  }

  public Command MoveToDataLevel()
  {
    return new FunctionalCommand(
      () -> this.ApplyPosition(DataStuff.GetLevel()),
      () -> {},
      interrupted -> this.LogClosedLoopResults(interrupted),
      () -> this.isReady(ElevatorConstants.POSITION_ERROR_TOLERANCE, ElevatorConstants.POSITION_DERIVATIVE_TOLERANCE),
      this
    );
  }

  public Command MoveToLevel(HEIGHTS height)
  {
    return new FunctionalCommand(
      () -> this.ApplyPosition(height),
      () -> {},
      interrupted -> this.LogClosedLoopResults(interrupted),
      () -> this.isReady(ElevatorConstants.POSITION_ERROR_TOLERANCE, ElevatorConstants.POSITION_DERIVATIVE_TOLERANCE),
      this
    );
  }

  // To-do: Move sysId settings to the constants file
  public SysIdRoutine BuildSysIdRoutine()
  {
    this.m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(0.25).per(Seconds),  // Ramp Rate in Volts / Seconds
         Volts.of(1), // Dynamic Step Voltage
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

  private void ApplyPosition(Level level)
  {
    HEIGHTS selectedHeight;
    switch(level)
    {
      case ONE:
        selectedHeight = HEIGHTS.ONE;
      break;

      case TWO:
        selectedHeight = HEIGHTS.TWO;
      break;

      case THREE:
        selectedHeight = HEIGHTS.THREE;
      break;

      case FOUR:
        selectedHeight = HEIGHTS.FOUR;
      break;

      default:
        selectedHeight = HEIGHTS.TRAVEL;
    }

    this.ApplyPosition(selectedHeight);
  }

  private void ApplyPosition(HEIGHTS height)
  {
    MotorManager.ApplyControlRequest(m_PositionRequest.withPosition(height.getValue()), Hardware.ELEVATOR_MOTOR);
    this.m_SelectedHeightPub.set(height.name());
    this.m_GoalPositionPub.set(height.getValue());
  }

  private void ApplyVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.ELEVATOR_MOTOR);
    this.m_SelectedHeightPub.set("VOLTAGE CONTROLLLLLLL");
    this.m_GoalPositionPub.set(9165493656.0);
  }

  private void Brake()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.ELEVATOR_MOTOR);
    this.m_SelectedHeightPub.set("BRAKING >:O");
    this.m_GoalPositionPub.set(9165493656.0);
  }

  private void Coast()
  {
    MotorManager.ApplyControlRequest(m_CoastRequest, Hardware.ELEVATOR_MOTOR);
    this.m_SelectedHeightPub.set("COASTING - Cruising the California Coast.");
    this.m_GoalPositionPub.set(9165493656.0);
  }

  private boolean isReady(double positionTolerance, double derivativeTolerance)
  {
    StatusSignal.refreshAll(this.m_ElevatorRPS, this.m_ElevatorPosition);
    if((Math.abs(this.m_ElevatorRPS.getValueAsDouble()) > derivativeTolerance) || (Math.abs(this.m_ElevatorPosition.getValueAsDouble() - this.m_PositionRequest.Position) > positionTolerance))
    {
      return false;
    }
    return true;
  }

  private void UpdateTelemetry()
  {
    StatusSignal.refreshAll(this.m_ElevatorPosition, this.m_ElevatorRPS);
    this.m_PositionPub.set(this.m_ElevatorPosition.getValueAsDouble());
    this.m_VelocityPub.set(this.m_ElevatorRPS.getValueAsDouble());
    this.m_InPositionPub.set(this.isReady(ElevatorConstants.POSITION_ERROR_TOLERANCE, ElevatorConstants.POSITION_DERIVATIVE_TOLERANCE));
  }

  private void LogClosedLoopResults(boolean interrupted)
  {
    StatusSignal.refreshAll(this.m_ElevatorPosition, this.m_ElevatorRPS);
    StringBuilder resultString = new StringBuilder();
    NumberFormat formatter = new DecimalFormat("#.###");
    if(interrupted)
    {
      resultString.append("Movement command INTERRUPTED.\n");
    }
    else
    {
      resultString.append("Movement command COMPLETED.\n");
    }
    
    resultString.append("Telemetry at finish:\n");
    resultString.append("Position: " + formatter.format(this.m_ElevatorPosition.getValueAsDouble()) + "\n");
    resultString.append("Velocity: " + formatter.format(this.m_ElevatorRPS.getValueAsDouble()) + "\n");
    resultString.append("Goal Position: " + formatter.format(this.m_PositionRequest.Position));

    this.m_Logger.Log(resultString.toString());
  }

  private void BuildToolbox()
  {
    SmartDashboard.putData("Brake Elevator", new InstantCommand(() -> this.Brake(), this).ignoringDisable(true));
    SmartDashboard.putData("Coast Elevator", new InstantCommand(() -> this.Coast(), this).ignoringDisable(true));
    SmartDashboard.putData("Elevator Voltage Up", new InstantCommand(() -> this.ApplyVoltage(1), this));
    SmartDashboard.putData("Elevator Voltage Down", new InstantCommand(() -> this.ApplyVoltage(-1), this));
  }
}