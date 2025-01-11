// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeCleanerConstants;
import frc.robot.constants.HardwareConstants;

/**
 * Creates a new Algae Cleaner subsystem.
 */
public class AlgaeCleanerSubsystem extends SubsystemBase 
{
  // Creates new Algae Cleaner roller and wrist motors.
  private final TalonFX m_AlgaeCleanerRollerMotor = new TalonFX(HardwareConstants.ALGAE_CLEANER_ROLLER_MOTOR_ID);
  private final TalonFX m_AlgaeCleanerWristMotor = new TalonFX(HardwareConstants.ALGAE_CLEANER_WRIST_MOTOR_ID);

  // Algae Cleaner subsystem constructor
  public AlgaeCleanerSubsystem() 
  {
    m_AlgaeCleanerRollerMotor.getConfigurator().apply(AlgaeCleanerConstants.ALGAE_CLEANER_ROLLER_GAINS);
    m_AlgaeCleanerWristMotor.getConfigurator().apply(AlgaeCleanerConstants.ALGAE_CLENAER_WRIST_CONFIGURATION);
  }

  /**
   * Sets the roller voltage for the Algae Cleaner roller motor.
   * @param cleanerVoltage Takes a paramater of voltage (how fast the motor rotates).
   */
  public void setCleanerRollerVoltage(double cleanerVoltage)
  {
    VoltageOut setCleanerVoltage = new VoltageOut(cleanerVoltage);
    m_AlgaeCleanerRollerMotor.setControl(setCleanerVoltage);
  }

  /**
   * Sets the wrist position for the Algae Cleaner roller motor.
   * @param cleanerPosition Takes a parameter of position (where cleaner arm is located in space).
   */
  public void setCleanerWristPosition(double cleanerPosition)
  {
    MotionMagicVoltage setCleanerPosition = new MotionMagicVoltage(cleanerPosition);
    m_AlgaeCleanerWristMotor.setControl(setCleanerPosition);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
