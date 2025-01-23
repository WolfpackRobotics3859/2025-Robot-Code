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
  private final TalonFX m_ClimbWristMotorMain = new TalonFX(Hardware.CLIMB_MAIN_WRIST_MOTOR_ID);
  private final TalonFX m_ClimbWristMotorFollower = new TalonFX(Hardware.CLIMB_FOLLOWER_WRIST_MOTOR_ID);
  
  /**
   * Climb subsystem constructor.
   */
  public Climb() 
  {
    m_ClimbWristMotorMain.getConfigurator().apply(ClimbConstants.CLIMB_MAIN_WRIST_CONFIGURATION);
    m_ClimbWristMotorFollower.getConfigurator().apply(ClimbConstants.CLIMB_FOLLOWER_WRIST_CONFIGURATION);

    // Sets the main climb wrist motor to follow the secondary climb wrist motor.
    Follower climbFollowRequest = new Follower(Hardware.CLIMB_MAIN_WRIST_MOTOR_ID, false); 
    m_ClimbWristMotorFollower.setControl(climbFollowRequest);
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

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
