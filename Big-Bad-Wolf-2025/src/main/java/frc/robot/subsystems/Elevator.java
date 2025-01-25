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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;

public class Elevator extends SubsystemBase
{
  private final TalonFX m_ElevatorMotorMain;
  private final TalonFX m_ElevatorMotorFollower;

  public final DigitalInput m_HallEffectSensor;// Curcuit has 0 volts if magnet is nearby, or else it has 5 volts

  /**
   * Constructor which runs anything in it upon initialization and creates a new object.
   */
  public Elevator()
  {
    m_ElevatorMotorMain = new TalonFX(Hardware.ELEVATOR_MOTOR_MAIN_ID);
    m_ElevatorMotorFollower = new TalonFX(Hardware.ELEVATOR_MOTOR_FOLLOWER_ID);

    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_MAIN_ID, false); 
    m_ElevatorMotorFollower.setControl(followRequest);// Sets follower motor to follow whatever main motor does

    m_HallEffectSensor = new DigitalInput(Hardware.HALL_EFFECT_DIO_PORT_ID);
  }

  /**
   * Uses Hall Effect Sensor to detect and return true when the elevator reaches the bottom.
   * Magnet is attached to bottom of elevator where Hall Effect sensor can open(turn off) curcuit when it detects it.
   * 
   * @return True if magnet on bottom of elevator reaches Hall Effect sensor. 
   */
  public boolean elevatorAtZeroPosition()
  {
    return this.m_HallEffectSensor.get();// .get() checks if curcuit is on or off (true if off)
  }

  /**
   * Used to zero encoder values so it knows its position.
   */
  public void zeroElevator()
  {
    if (elevatorAtZeroPosition())
    {
      // If elevator is at zero position, set encoder value to 0
      m_ElevatorMotorMain.setPosition(0);
    }
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
    m_ElevatorMotorMain.setControl(voltageRequest);
  }

  /** 
   * Sets elevator motor to work at a percentage of the max power it can.
   * @param percentage percentage of max power motor works at
   */
  private void setElevatorPercentage(double percentage)
  {
    DutyCycleOut dutyCycleRequest = new DutyCycleOut(percentage);
    m_ElevatorMotorMain.setControl(dutyCycleRequest);
  }

  /** 
   * Sets elevator position
   * @param position as an angle or value depending on motor purpose
   */
  private void setElevatorPosition(double position)
  {
    MotionMagicVoltage positionRequest = new MotionMagicVoltage(position);
    m_ElevatorMotorMain.setControl(positionRequest);
  }

  /**
   * Stops movement and discourages any further movement from external forces
   */
  private void brakeElevator()
  {
    StaticBrake brakeRequest = new StaticBrake();
    m_ElevatorMotorMain.setControl(brakeRequest);
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    zeroElevator();// Zeros elevator whenever elevator is at the zero position
  }
}
