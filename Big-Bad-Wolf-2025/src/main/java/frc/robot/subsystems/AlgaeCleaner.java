// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeCleanerConstants;
import frc.robot.constants.Hardware;

// Algae Cleaner subsystem.
public class AlgaeCleaner extends SubsystemBase 
{
  private final TalonFX m_AlgaeCleanerShooterMotor = new TalonFX(Hardware.ALGAE_CLEANER_SHOOTER_MOTOR_ID);
  private final TalonFX m_AlgaeCleanerWristMotor = new TalonFX(Hardware.ALGAE_CLEANER_WRIST_MOTOR_ID);

  /**
   *  Algae Cleaner subsystem constructor.
   */
  public AlgaeCleaner() 
  {
    m_AlgaeCleanerShooterMotor.getConfigurator().apply(AlgaeCleanerConstants.ALGAE_CLEANER_ROLLER_GAINS);
    m_AlgaeCleanerWristMotor.getConfigurator().apply(AlgaeCleanerConstants.ALGAE_CLENAER_WRIST_CONFIGURATION);
  }

  /**
   * Sets the shooter voltage for the Algae Cleaner shooter motor.
   * @param cleanerVoltage Assigns voltage to the motors (amount of applicable torque).
   */
  public void setCleanerRollerVoltage(double cleanerVoltage)
  {
    m_AlgaeCleanerShooterMotor.setControl(new VoltageOut(cleanerVoltage));
  }

  /**
   * Sets the wrist position for the Algae Cleaner motor.
   * @param cleanerPosition Assigns wrist position to the motors (where cleaner arm is located in space).
   */
  public void setCleanerWristPosition(double cleanerPosition)
  {
    m_AlgaeCleanerWristMotor.setControl(new MotionMagicVoltage(cleanerPosition));
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
