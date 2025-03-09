// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
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
    MotorManager.AddMotor("CLIMB MOTOR MAIN", Hardware.CLIMB_WRIST_MOTOR_MAIN);
    MotorManager.AddMotor("CLIMB MOTOR FOLLOWER", Hardware.CLIMB_WRIST_MOTOR_FOLLOWER);
    MotorManager.AddMotor("FUNNEL LATCH MOTOR", Hardware.CORAL_FUNNEL_MOTOR);

    MotorManager.ApplyConfigs(ClimbConstants.CLIMB_WRIST_MAIN_CONFIGURATION, Hardware.CLIMB_WRIST_MOTOR_MAIN);
    MotorManager.ApplyConfigs(ClimbConstants.CLIMB_WRIST_FOLLOWER_CONFIGURATION, Hardware.CLIMB_WRIST_MOTOR_FOLLOWER);
    MotorManager.ApplyConfigs(ClimbConstants.FUNNEL_LATCH_MOTOR_CONFIGURATION, Hardware.CORAL_FUNNEL_MOTOR);

    Follower climbFollowRequest = new Follower(Hardware.CLIMB_WRIST_MOTOR_MAIN, false); 
    MotorManager.ApplyControlRequest(climbFollowRequest, Hardware.CLIMB_WRIST_MOTOR_FOLLOWER);

    m_VoltageRequest = new VoltageOut(0);
  }

  public Command setClimbVoltage(DoubleSupplier voltageSupplier)
  {
    return this.run(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltageSupplier.getAsDouble()), Hardware.CLIMB_WRIST_MOTOR_MAIN));
  }

  public Command setLatchVoltage(double voltage)
  {
    return this.runOnce(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CORAL_FUNNEL_MOTOR));
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}