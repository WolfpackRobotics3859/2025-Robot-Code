// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.AlgaeIntakeConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.constants.Hardware;

// Creates a new Algae Intake subsystem.
public class AlgaeIntakeSubsystem extends SubsystemBase 
{
  private final TalonFX m_AlgaeIntakeRollerMotor = new TalonFX(Hardware.ALGAE_INTAKE_ROLLER_MOTOR_ID);
  private final TalonFX m_AlgaeIntakeWristMotor = new TalonFX(Hardware.ALGAE_INTAKE_WRIST_MOTOR_ID);

  /**
   * Algae Intake subsystem constructor.
   */
  public AlgaeIntakeSubsystem() 
  {
    m_AlgaeIntakeRollerMotor.getConfigurator().apply(AlgaeIntakeConstants.ALGAE_INTAKE_ROLLER_GAINS);
    m_AlgaeIntakeWristMotor.getConfigurator().apply(AlgaeIntakeConstants.ALGAE_INTAKE_WRIST_CONFIGURATION);
  }

  /**
   * Sets the roller voltage for the Algae Intake roller motor.
   * 
   * @param voltage Assigns voltage to the motors (amount of applicable torque).
   */
  public void setRollerVoltage(double voltage)
  {
    VoltageOut setVoltage = new VoltageOut(voltage);
    m_AlgaeIntakeRollerMotor.setControl(setVoltage);
  }

  /**
   * Sets the wrist position for the Algae Intake wrist motor.
   * 
   * @param position Assigns wrist position to the motors (where intake arm is located in space).
   */
  public void setWristPosition(double position)
  {
    MotionMagicVoltage setPosition = new MotionMagicVoltage(position);
    m_AlgaeIntakeWristMotor.setControl(setPosition);
  }

  @Override
  public void periodic() 
  {
    // Intentionally Empty.
  }
}