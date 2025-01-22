// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.HardwareConstants;

/**
 * Creates a new Climb subsystem.
 */
public class ClimbSubsystem extends SubsystemBase 
{
  // Creates a new Climb wrist motor.
  private final TalonFX m_ClimbWristMotor = new TalonFX(HardwareConstants.CLIMB_WRIST_MOTOR_ID);

  /**
   * Climb subsystem constructor.
   */
  public ClimbSubsystem() 
  {
    m_ClimbWristMotor.getConfigurator().apply(ClimbConstants.CLIMB_WRIST_CONFIGURATION);
  }

  /**
   * Sets the wrist position for the Climb wrist motor.
   * @param climbPosition Takes a parameter of position (where climb arm is located in space).
   */
  public void setClimbWristPosition(double climbPosition)
  {
    MotionMagicVoltage setClimbPosition = new MotionMagicVoltage(climbPosition);
    m_ClimbWristMotor.setControl(setClimbPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
