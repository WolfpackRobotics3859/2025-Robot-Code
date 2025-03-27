// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.Hardware;
import frc.robot.utilities.MotorManager;

// Creates a new Climb subsystem.
public class Climb extends SubsystemBase
{ 
  private final VoltageOut m_VoltageRequest;
  private final MotionMagicVoltage m_PositionRequest;

  private final StatusSignal<Angle> m_PositionSignal;

  private final TalonFX m_WristMotor;

  private final TimeOfFlight m_TOF;

  /**
   * Climb subsystem constructor.
   */
  public Climb() 
  {
    MotorManager.AddMotor("CLIMB WRIST MOTOR", Hardware.CLIMB_WRIST_MOTOR_ID);
    MotorManager.AddMotor("CLIMB ROLLER MOTOR", Hardware.CLIMB_ROLLER_MOTOR_ID);

    this.m_TOF = new TimeOfFlight(Hardware.CLIMB_TOF_SENSOR);
    this.m_TOF.setRangingMode(RangingMode.Short, 100);
    this.m_TOF.setRangeOfInterest(8, 8, 12, 12);

    MotorManager.ApplyConfigs(ClimbConstants.WRIST_MOTOR_CONFIG, Hardware.CLIMB_WRIST_MOTOR_ID);
    MotorManager.ApplyConfigs(ClimbConstants.ROLLER_MOTOR_CONFIG, Hardware.CLIMB_ROLLER_MOTOR_ID);

    this.m_WristMotor = MotorManager.GetMotor(Hardware.CLIMB_WRIST_MOTOR_ID);
    this.m_PositionSignal = this.m_WristMotor.getPosition();

    m_VoltageRequest = new VoltageOut(0);
    m_PositionRequest = new MotionMagicVoltage(0);
  }

  public Command setClimbVoltage(double voltage)
  {
    return this.runOnce(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_WRIST_MOTOR_ID));
  }

  public Command ApplyRollerVoltage(double voltage)
  {
    return this.runOnce(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_ROLLER_MOTOR_ID));
  }

  public Command SeekCage()
  {
        return new FunctionalCommand(() -> {
          this.setRollerVoltage(ClimbConstants.CLIMB_ROLLER_VOLTAGE);
          this.SetWristPosition(ClimbConstants.CLIMB_TAKING_POSITION);
        },
                                     () -> {},
                                     interrupted -> 
                                     {
                                        this.setRollerVoltage(0);
                                     }, 
                                     () -> false,
                                     this);
  }

  public Command DefaultSafety()
  {
        return new FunctionalCommand(() -> {},
                                     () -> 
                                     {
                                      if(this.m_PositionSignal.refresh().getValueAsDouble() > ClimbConstants.CLIMB_STOP_POSITION)
                                      {
                                        this.setClimbVoltage(0);
                                      }
                                     },
                                     interrupted -> 
                                     {}, 
                                     () -> false,
                                     this);
  }

  public Command GoWristPosition(double position)
  {
    return this.runOnce(() -> this.SetWristPosition(position));
  }

  private boolean CageInRange()
  {
    return this.m_TOF.getRange() < ClimbConstants.TOF_IN_RANGE_THRESHOLD;
  }

  private void setRollerVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_ROLLER_MOTOR_ID);
  }

  private void SetWristPosition(double position)
  {
     MotorManager.ApplyControlRequest(m_PositionRequest.withPosition(position), Hardware.CLIMB_WRIST_MOTOR_ID);
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Climb Range", this.m_TOF.getRange());
    // This method will be called once per scheduler run
  }
}