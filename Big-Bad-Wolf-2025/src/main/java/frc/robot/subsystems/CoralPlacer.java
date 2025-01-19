// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;

public class CoralPlacer extends SubsystemBase
{
  private final TalonFX m_CoralPlacerMotor;
  private final TalonFX m_CoralFunnelMotor;
  /**
   * Coral Placer subsystem.
  */
  public CoralPlacer()
  {
    m_CoralPlacerMotor = new TalonFX(Hardware.CORAL_PLACER_MOTOR);
    m_CoralFunnelMotor = new TalonFX(Hardware.CORAL_FUNNEL_MOTOR_ID);
  }
  
  /**
   * Sets the control mode and value input calling appropriate methods.
   * @param controlMode Mode to set motor input
   * @param value motor input
   */
  public void CoralPlacerRequest(Global.MODE controlMode, double value)
  {
    switch(controlMode)
    {
      case VOLTAGE:
        setPlacerVoltage(value);
        break;
      case PERCENT:
        setPlacerPercentage(value);
        break;
      case POSITION:
        System.out.print("MODE: POSITION IS NOT AVAILABLE FOR CORAL SHOOTER");
        break;
      case BRAKE:
        System.out.print("MODE: BRAKE IS NOT AVAILABLE FOR CORAL SHOOTER");
        break;
    }
  }

  /** 
   * Sets voltage for placer motor.
   * @param voltage amount of volts
   */
  private void setPlacerVoltage(double voltage)
  {
    VoltageOut voltageRequest = new VoltageOut(voltage);
    m_CoralPlacerMotor.setControl(voltageRequest);
  }

  /** 
   * Sets placer motor to work at a percentage of the max power it can.
   * @param percentage percentage of max power motor works at
   */
  private void setPlacerPercentage(double percentage)
  {
    DutyCycleOut dutyCycleRequest = new DutyCycleOut(percentage);
    m_CoralPlacerMotor.setControl(dutyCycleRequest);
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
