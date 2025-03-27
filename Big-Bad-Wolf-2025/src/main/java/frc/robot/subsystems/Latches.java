// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.Hardware;
import frc.robot.utilities.MotorManager;

public class Latches extends SubsystemBase 
{
  private VoltageOut m_VoltageRequest;

  /** Creates a new Latches. */
  public Latches() 
  {
    MotorManager.AddMotor("FUNNEL LATCH MOTOR", Hardware.CLIMB_RELEASE_MOTOR_ID);

    MotorManager.ApplyConfigs(ClimbConstants.FEET_MOTOR_CONFIG, Hardware.CLIMB_RELEASE_MOTOR_ID);

    m_VoltageRequest = new VoltageOut(0);
  }

  public Command setLatchVoltage(double voltage)
  {
    return this.runOnce(() -> MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.CLIMB_RELEASE_MOTOR_ID));
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
