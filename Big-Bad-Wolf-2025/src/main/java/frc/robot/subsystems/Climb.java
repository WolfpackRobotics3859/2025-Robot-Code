// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.Hardware;

// Creates a new Climb subsystem.
public class Climb extends SubsystemBase
{
  private final TalonFX m_ClimbWristMotorMain = new TalonFX(Hardware.CLIMB_WRIST_MOTOR_MAIN_ID);
  private final TalonFX m_ClimbWristMotorFollower = new TalonFX(Hardware.CLIMB_WRIST_MOTOR_FOLLOWER_ID);

  private final TalonFX m_CoralFunnelMotor;
  
  /**
   * Climb subsystem constructor.
   */
  public Climb() 
  {
    m_ClimbWristMotorMain.getConfigurator().apply(ClimbConstants.CLIMB_WRIST_MAIN_CONFIGURATION);
    m_ClimbWristMotorFollower.getConfigurator().apply(ClimbConstants.CLIMB_WRIST_FOLLOWER_CONFIGURATION);

    // Sets the main climb wrist motor to follow the secondary climb wrist motor.
    Follower climbFollowRequest = new Follower(Hardware.CLIMB_WRIST_MOTOR_MAIN_ID, false); 
    m_ClimbWristMotorFollower.setControl(climbFollowRequest);

    m_CoralFunnelMotor = new TalonFX(Hardware.CORAL_FUNNEL_MOTOR_ID);
  }

  /**
   * Sets the wrist position for the Climb wrist motor.
   * @param climbPosition Assigns wrist position to the motors (where climb arm is located in space).
   */
  public void setClimbWristPosition(double climbPosition)
  {
    MotionMagicVoltage climbPositionRequest = new MotionMagicVoltage(climbPosition);
    m_ClimbWristMotorMain.setControl(climbPositionRequest);
  }

  /**
   * Sets the wrist position for the funnel wrist motor.
   * @param funnelPosition Position to set the funnel wrist to.
   */
  public void setFunnelWristPosition(double funnelPosition)
  {
    MotionMagicVoltage positionRequest = new MotionMagicVoltage(funnelPosition);
    m_CoralFunnelMotor.setControl(positionRequest);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}