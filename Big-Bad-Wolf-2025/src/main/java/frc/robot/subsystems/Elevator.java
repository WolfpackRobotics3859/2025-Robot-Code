// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;

public class Elevator extends SubsystemBase
{
  private final TalonFX m_ElevatorMotor1;
  private final TalonFX m_ElevatorMotor2;

  /**
   * Constructor which runs anything in it upon initialization and creates a new object.
   */
  public Elevator()
  {
    m_ElevatorMotor1 = new TalonFX(Hardware.ELEVATOR_MOTOR_1_ID);
    m_ElevatorMotor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_2_ID);

    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_1_ID, false); 
    m_ElevatorMotor2.setControl(followRequest);// Sets motor 2 to follow whatever motor 1 does
  }

  /**
   * Moves elevator based on what control mode and value is chosen.
   * 
   * @param controlMode Which type of mode to use for motor.
   * @param value value fed as an argument into chosen method
   */
  public void elevatorRequest(Global.MODE controlMode, double value)
  {
    switch(controlMode)
    {
      case VOLTAGE:
        setElevatorVoltage(value);
        break;
      case PERCENT:
        setElevatorPercentage(value);
        break;
      case POSITION:
        setElevatorPosition(value);
        break;
      case BRAKE:
        brakeElevator();
        break;
    }
  }

  /** 
   * Sets voltage for elevator motor
   * 
   * @param voltage amount of volts
   */
  private void setElevatorVoltage(double voltage)
  {
    VoltageOut voltageRequest = new VoltageOut(voltage);
    m_ElevatorMotor1.setControl(voltageRequest);
  }

  /** 
   * Sets elevator motor to work at a percentage of the max power it can.
   * @param percentage percentage of max power motor works at
   */
  private void setElevatorPercentage(double percentage)
  {
    DutyCycleOut dutyCycleRequest = new DutyCycleOut(percentage);
    m_ElevatorMotor1.setControl(dutyCycleRequest);
  }

  /** 
   * Sets elevator position
   * @param position as an angle or value depending on motor purpose
   */
  private void setElevatorPosition(double position)
  {
    MotionMagicVoltage positionRequest = new MotionMagicVoltage(position);
    m_ElevatorMotor1.setControl(positionRequest);
  }

  /**
   * Stops movement and discourages any further movement from external forces
   */
  private void brakeElevator()
  {
    StaticBrake brakeRequest = new StaticBrake();
    m_ElevatorMotor1.setControl(brakeRequest);
  }

  

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
