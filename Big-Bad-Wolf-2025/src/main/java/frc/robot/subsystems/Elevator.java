// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.subsystems.elevator.ElevatorRequest;

public class Elevator extends SubsystemBase
{
  // Hardware
  private final TalonFX m_ElevatorMotorMain;
  private final TalonFX m_ElevatorMotorFollower;
  private final DigitalInput m_HallEffectSensor;
  
  private ElevatorRequest m_CurrentRequest;
  private boolean hasZeroed = false;

  /**
   * Constructor which runs anything in it upon initialization and creates a new object.
   */
  public Elevator()
  {
    m_ElevatorMotorMain = new TalonFX(Hardware.ELEVATOR_MOTOR_MAIN_ID);
    m_ElevatorMotorFollower = new TalonFX(Hardware.ELEVATOR_MOTOR_FOLLOWER_ID);

    m_ElevatorMotorMain.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    m_ElevatorMotorFollower.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_MAIN_ID, false); 
    m_ElevatorMotorFollower.setControl(followRequest);// Sets follower motor to follow whatever main motor does

    m_HallEffectSensor = new DigitalInput(Hardware.HALL_EFFECT_DIO_PORT_ID);
  }

  public void ApplyRequest(ElevatorRequest request)
  {
    ControlRequest motorRequest = null;

    switch(request.GetType())
    {
      case NOOP:
        if(this.m_CurrentRequest == null)
        {
          this.ApplyRequest(new ElevatorRequest().PercentOutput(0)); // If there is no previous request then create a neutral one. Otherwise, maintain previous request.
          return;
        }
      break;

      case BRAKE:
      {
        motorRequest = new StaticBrake();
      }
      break;

      case VOLTAGE:
      {
        motorRequest = new VoltageOut(request.GetValue());
      }
      break;

      case PERCENT:
      {
        motorRequest = new DutyCycleOut(request.GetValue());
      }
      break;

      case POSITION:
      {
        motorRequest = new MotionMagicVoltage(request.GetValue());
      }
      break;
    }

    if(motorRequest == null)
    {
      System.out.println("ERROR: Attempted to assign a null control request to the elevator motors.");
    }
    else
    {
      SmartDashboard.putString("[Elevator] Current Request Type", request.GetType().name());
      SmartDashboard.putNumber("[Elevator] Goal Value", request.GetValue());
      this.m_ElevatorMotorMain.setControl(motorRequest);
    }
  }

  public double getElevatorPosition()
  {
    return this.m_ElevatorMotorMain.getPosition().getValueAsDouble();
  }

  private void zeroElevator()
  {
    if (!this.m_HallEffectSensor.get())
    {
      if(!hasZeroed)
      {
        m_ElevatorMotorMain.setPosition(0);
        this.hasZeroed = true;
      }
    }
    else
    {
      this.hasZeroed = false;
    }
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    this.zeroElevator();// Zeros elevator whenever elevator is at the zero position
    SmartDashboard.putNumber("Elevator Position", this.getElevatorPosition());
    SmartDashboard.putBoolean("Elevator Hall Effect Sensor", !this.m_HallEffectSensor.get());
  }

  private void InitializeSmartdashboardFields()
  {
    SmartDashboard.putString("[Elevator] Current Request Type", "N/A");
    SmartDashboard.putNumber("[Elevator] Goal Value", 0);
  }
}
