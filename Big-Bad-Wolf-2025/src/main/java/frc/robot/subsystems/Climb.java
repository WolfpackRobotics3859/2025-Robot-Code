// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.Hardware;
import frc.robot.utilities.MotorManager;

// Creates a new Climb subsystem.
public class Climb extends SubsystemBase
{ 
  private final VoltageOut m_VoltageRequest;
  private final TalonFX m_LatchMotor;
  private final TalonFX m_FootMotor;

  /**
   * Climb subsystem constructor.
   */
  public Climb() 
  {
    MotorManager.AddMotor("CLIMB WRIST MOTOR", Hardware.CLIMB_WRIST_MOTOR);
    MotorManager.AddMotor("CLIMB ROLLER MOTOR", Hardware.CLIMB_ROLLER_MOTOR);
    MotorManager.AddMotor("FUNNEL LATCH MOTOR", Hardware.CORAL_FUNNEL_MOTOR);

    MotorManager.ApplyConfigs(ClimbConstants.CLIMB_WRIST_CONFIGURATION, Hardware.CLIMB_WRIST_MOTOR);
    MotorManager.ApplyConfigs(ClimbConstants.CLIMB_ROLLER_CONFIGURATION, Hardware.CLIMB_ROLLER_MOTOR);
    MotorManager.ApplyConfigs(ClimbConstants.FUNNEL_LATCH_MOTOR_CONFIGURATION, Hardware.CORAL_FUNNEL_MOTOR);

    m_VoltageRequest = new VoltageOut(0);
    m_LatchMotor = MotorManager.GetMotor(Hardware.CORAL_FUNNEL_MOTOR);
    m_FootMotor = MotorManager.GetMotor(Hardware.CORAL_FUNNEL_MOTOR);


  }

  public Command setClimbVoltage(double voltage)
  {
    return this.run(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_WRIST_MOTOR));
  }

  public Command setRollerVoltage(double voltage)
  {
    return this.run(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_ROLLER_MOTOR));
  }

  public Command setLatchPosition(double position)
  {
    return this.runOnce(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(position), Hardware.CORAL_FUNNEL_MOTOR));
  }

  public Command setFootPosition(double position)
  {
    return this.runOnce(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(position), Hardware.CORAL_FUNNEL_MOTOR));
  }

  @Override
  public void periodic() 
  {
  SmartDashboard.putNumber("Latch Position", this.m_LatchMotor.getPosition().getValueAsDouble());
  SmartDashboard.putNumber("Foot Position", this.m_FootMotor.getPosition().getValueAsDouble());  
  }
}