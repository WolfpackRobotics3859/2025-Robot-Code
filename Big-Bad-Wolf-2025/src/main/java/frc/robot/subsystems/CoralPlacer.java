// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;

public class CoralPlacer extends SubsystemBase
{
  private final TalonFX m_CoralPlacerMotor = new TalonFX(Hardware.CORAL_PLACER_MOTOR);
  /**Constructor: When a Coral placer is instantiated everything in the constructor runs
   * currently no parameters
  */
  public CoralPlacer()
  {

  }
  
  /**based on the control mode and value input into the parameters will call a method through a command
   * @param controlMode
   * @param value
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

  /** Sets voltage for placer motor
   * @param voltage amount of volts
   */
  private void setPlacerVoltage(double voltage)
  {
    VoltageOut voltageRequest = new VoltageOut(voltage);
    m_CoralPlacerMotor.setControl(voltageRequest);
  }

  /** Sets placer motor to work at a percentage of the max power it can.
   * @param percentage percentage of max power motor works at
   */
  private void setPlacerPercentage(double percentage)
  {
    DutyCycleOut dutyCycleRequest = new DutyCycleOut(percentage);
    m_CoralPlacerMotor.setControl(dutyCycleRequest);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
