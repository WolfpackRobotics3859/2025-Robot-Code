// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Hardware;
import frc.robot.subsystems.elevator.ElevatorRequest;

public class Elevator extends SubsystemBase
{
  // Hardware
  private final TalonFX m_ElevatorMotorMain;
  private final TalonFX m_ElevatorMotorFollower;
  
  private ElevatorRequest m_CurrentRequest;
  private SysIdRoutine m_SysIdRoutine;

  /**
   * Constructor which runs anything in it upon initialization and creates a new object.
   */
  public Elevator()
  {
    m_ElevatorMotorMain = new TalonFX(Hardware.ELEVATOR_MOTOR_LEFT_ID);
    m_ElevatorMotorFollower = new TalonFX(Hardware.ELEVATOR_MOTOR_RIGHT_ID);

    m_ElevatorMotorMain.getConfigurator().apply(ElevatorConstants.LEFT_MOTOR_CONFIG);
    m_ElevatorMotorFollower.getConfigurator().apply(ElevatorConstants.RIGHT_MOTOR_CONFIG);

    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_LEFT_ID, false); 
    m_ElevatorMotorFollower.setControl(followRequest);

    this.initializeSmartdashboardFields();
  }

  public Command goToPosition(double position)
  {
    return this.runOnce(() -> this.ApplyRequest(new ElevatorRequest().Position(position)));
  }

  public Command applyElevatorVoltage(double voltage)
  {
    return this.runOnce(() -> this.ApplyRequest(new ElevatorRequest().VoltageOut(voltage)));
  }

  public Command brakeElevator()
  {
    return this.runOnce(() -> this.ApplyRequest(new ElevatorRequest().Brake()));
  }

  public double getElevatorPosition()
  {
    return this.m_ElevatorMotorMain.getPosition().getValueAsDouble();
  }

  public SysIdRoutine getSysIdRoutine()
  {
    return this.m_SysIdRoutine;
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

  public SysIdRoutine BuildSysIdRoutine()
  {
    this.m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
         null,         // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,          // Use default timeout (10 s)
         (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_ElevatorMotorMain.setControl(new VoltageOut(volts.in(Volts))),
         null,
         this
      )
   );
   return this.m_SysIdRoutine;
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Elevator Position", this.getElevatorPosition());
  }

  private void initializeSmartdashboardFields()
  {
    SmartDashboard.putString("[Elevator] Current Request Type", "N/A");
    SmartDashboard.putNumber("[Elevator] Goal Value", 0);
  }
}
