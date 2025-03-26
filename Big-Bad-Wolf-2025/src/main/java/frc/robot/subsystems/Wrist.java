// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.constants.WristConstants.ANGLES;
import frc.robot.lib.Positions.Level;
import frc.robot.constants.Hardware;
import frc.robot.constants.WristConstants;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.MotorManager;
import frc.robot.utilities.PackLog;

public class Wrist extends SubsystemBase 
{
  private final PackLog m_Logger;

  private final TalonFX m_WristMotor;
  private final CANcoder m_Encoder;
  private SysIdRoutine m_SysIdRoutine;

  private final MotionMagicVoltage m_WristPositionRequest;
  private final StaticBrake m_BrakeRequest;
  private final CoastOut m_CoastRequest;

  private final StatusSignal<AngularVelocity> m_WristRPS;
  private final StatusSignal<Angle> m_WristPosition;

  private final NetworkTable m_Table;

  private final DoublePublisher m_PositionPub;
  private final DoublePublisher m_VelocityPub;
  private final DoublePublisher m_GoalPositionPub;
  private final StringPublisher m_SelectedAnglePub;
  private final BooleanPublisher m_InPositionPub;

  public Wrist()
  {
    this.m_Logger = new PackLog(WristConstants.NAME);

    MotorManager.AddMotor("WRIST MOTOR", Hardware.WRIST_MOTOR_ID);
    this.m_WristMotor = MotorManager.GetMotor(Hardware.WRIST_MOTOR_ID);
    MotorManager.ApplyConfigs(WristConstants.WRIST_MOTOR_CONFIG, Hardware.WRIST_MOTOR_ID);

    this.m_Encoder = new CANcoder(Hardware.WRIST_ENCODER);

    this.m_WristPositionRequest = new MotionMagicVoltage(0);
    this.m_BrakeRequest = new StaticBrake();
    this.m_CoastRequest = new CoastOut();

    this.m_WristRPS = this.m_WristMotor.getVelocity();
    this.m_WristPosition = this.m_WristMotor.getPosition();

    this.m_Table = NetworkTableInstance.getDefault().getTable(WristConstants.NAME);
    this.m_PositionPub = m_Table.getDoubleTopic("Position").publish();
    this.m_VelocityPub = m_Table.getDoubleTopic("Velocity").publish();
    this.m_GoalPositionPub = m_Table.getDoubleTopic("Goal Position").publish();
    this.m_SelectedAnglePub = m_Table.getStringTopic("Selected Angle").publish();
    this.m_InPositionPub = m_Table.getBooleanTopic("In Position").publish();

    this.BuildToolbox();
  }

  public Command MoveToAngle(ANGLES angle)
  {
    return new FunctionalCommand(() -> this.ApplyAngle(angle),
                                 () -> {}, 
                                 interrupted -> this.LogClosedLoopResults(interrupted),
                                 () -> this.isReady(WristConstants.WRIST_POSITION_ERROR_TOLERANCE, WristConstants.WRIST_POSITION_DERIVATIVE_TOLERANCE),
                                 this);
  }

  public Command MoveToSelectedCoralDeployment()
  {
    return new FunctionalCommand(() -> this.MoveToProperCoralShotAccordingToData(),
                                 () -> {}, 
                                 interrupted -> this.LogClosedLoopResults(interrupted),
                                 () -> this.isReady(WristConstants.WRIST_POSITION_ERROR_TOLERANCE, WristConstants.WRIST_POSITION_DERIVATIVE_TOLERANCE),
                                 this);
  }

  @Override
  public void periodic() 
  {
    this.UpdateTelemetry();
  }

    // To-do: Move sysId settings to the constants file
  public SysIdRoutine BuildSysIdRoutine()
    {
      this.m_SysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
           Volts.of(0.2).per(Seconds),         // Use default ramp rate (1 V/s)
           Volts.of(0.3), // Reduce dynamic step voltage to 4 to prevent brownout
           null,          // Use default timeout (10 s)
           (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
        ),
        new SysIdRoutine.Mechanism(
           (volts) -> m_WristMotor.setControl(new VoltageOut(volts.in(Volts))),
           null,
           this
        )
     );
     return this.m_SysIdRoutine;
  }

  private void MoveToProperCoralShotAccordingToData()
  {
    if(DataStuff.GetLevel() == Level.FOUR)
    {
      this.ApplyAngle(WristConstants.ANGLES.DEPLOY_HIGH);
    }
    else
    {
      this.ApplyAngle(WristConstants.ANGLES.DEPLOY_LOW);
    }
  }

  private void ApplyAngle(ANGLES angle)
  {
    MotorManager.ApplyControlRequest(this.m_WristPositionRequest.withPosition(angle.getValue()), Hardware.WRIST_MOTOR_ID);
    this.m_SelectedAnglePub.set(angle.name());
    this.m_GoalPositionPub.set(angle.getValue());
  }

  private void ApplyBrake()
  {
    MotorManager.ApplyControlRequest(this.m_BrakeRequest, Hardware.WRIST_MOTOR_ID);
    this.m_SelectedAnglePub.set("BRAKE - Call the firefighters.");
    this.m_GoalPositionPub.set(9165493656.0);
  }

  private void ApplyCoast()
  {
    MotorManager.ApplyControlRequest(this.m_CoastRequest, Hardware.WRIST_MOTOR_ID);
    this.m_SelectedAnglePub.set("COASTING - Cruising the California Coast.");
    this.m_GoalPositionPub.set(9165493656.0);
  }

  private boolean isReady(double positionTolerance, double derivativeTolerance)
  {
    StatusSignal.refreshAll(this.m_WristRPS, this.m_WristPosition);
    if((Math.abs(this.m_WristRPS.getValueAsDouble()) > derivativeTolerance) || (Math.abs(this.m_WristPosition.getValueAsDouble() - this.m_WristPositionRequest.Position) > positionTolerance))
    {
      return false;
    }
    return true;
  }

  private void LogClosedLoopResults(boolean interrupted)
  {
    StatusSignal.refreshAll(this.m_WristPosition, this.m_WristRPS);
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
    resultString.append("Position: " + formatter.format(this.m_WristPosition.getValueAsDouble()) + "\n");
    resultString.append("Velocity: " + formatter.format(this.m_WristRPS.getValueAsDouble()) + "\n");
    resultString.append("Goal Position: " + formatter.format(this.m_WristPositionRequest.Position));

    this.m_Logger.Log(resultString.toString());
  }

  private void UpdateTelemetry()
  {
    StatusSignal.refreshAll(this.m_WristPosition, this.m_WristRPS);
    this.m_PositionPub.set(this.m_WristPosition.getValueAsDouble());
    this.m_VelocityPub.set(this.m_WristRPS.getValueAsDouble());
    this.m_InPositionPub.set(this.isReady(WristConstants.WRIST_POSITION_ERROR_TOLERANCE, WristConstants.WRIST_POSITION_DERIVATIVE_TOLERANCE));
  }

  private void BuildToolbox()
  {
    SmartDashboard.putData("Brake Wrist", new InstantCommand(() -> this.ApplyBrake(), this).ignoringDisable(true));
    SmartDashboard.putData("Coast Wrist", new InstantCommand(() -> this.ApplyCoast(), this).ignoringDisable(true));
  }
}
