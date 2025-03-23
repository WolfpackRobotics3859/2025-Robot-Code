// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.Hardware;
import frc.robot.utilities.MotorManager;

// Creates a new Climb subsystem.
public class Climb extends SubsystemBase
{ 
  private final VoltageOut m_VoltageRequest;

  /**
   * Climb subsystem constructor.
   */
  public Climb() 
  {
    MotorManager.AddMotor("CLIMB WRIST MOTOR", Hardware.CLIMB_WRIST_MOTOR_ID);
    MotorManager.AddMotor("CLIMB ROLLER MOTOR", Hardware.CLIMB_ROLLER_MOTOR_ID);
    MotorManager.AddMotor("FUNNEL LATCH MOTOR", Hardware.CLIMB_RELEASE_MOTOR_ID);

    MotorManager.ApplyConfigs(ClimbConstants.WRIST_MOTOR_CONFIG, Hardware.CLIMB_WRIST_MOTOR_ID);
    MotorManager.ApplyConfigs(ClimbConstants.ROLLER_MOTOR_CONFIG, Hardware.CLIMB_ROLLER_MOTOR_ID);
    MotorManager.ApplyConfigs(ClimbConstants.FEET_MOTOR_CONFIG, Hardware.CLIMB_RELEASE_MOTOR_ID);

    m_VoltageRequest = new VoltageOut(0);
  }

  public Command setClimbVoltage(double voltage)
  {
    return this.run(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_WRIST_MOTOR_ID));
  }

  public Command setRollerVoltage(double voltage)
  {
    return this.run(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_ROLLER_MOTOR_ID));
  }

  public Command setLatchVoltage(double voltage)
  {
    return this.runOnce(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_RELEASE_MOTOR_ID));
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}