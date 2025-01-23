// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.Hardware;

// Creates a new Climb subsystem.
public class Climb extends SubsystemBase
{
  private final TalonFX m_ClimbWristMotor = new TalonFX(Hardware.CLIMB_MOTOR_ID);
  
  /**
   * Climb subsystem constructor.
   */
  public Climb() 
  {
    m_ClimbWristMotor.getConfigurator().apply(ClimbConstants.CLIMB_WRIST_CONFIGURATION);
  }

  /**
   * Sets the wrist position for the Climb wrist motor.
   * @param climbPosition Assigns wrist position to the motors (where climb arm is located in space).
   */
  public void setClimbWristPosition(double climbPosition)
  {
    MotionMagicVoltage climbPositionRequest = new MotionMagicVoltage(climbPosition);
    m_ClimbWristMotor.setControl(climbPositionRequest);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
