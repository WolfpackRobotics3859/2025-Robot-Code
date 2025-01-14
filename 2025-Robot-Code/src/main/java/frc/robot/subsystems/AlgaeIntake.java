// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.constants.HardwareConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

/**
 * Creates a new Algae Intake subsystem.
 */
public class AlgaeIntake extends SubsystemBase 
{
  // Creates new Algae Intake roller and wrist motors.
  private final TalonFX m_AlgaeIntakeRollerMotor = new TalonFX(HardwareConstants.ALGAE_INTAKE_ROLLER_MOTOR_ID);
  private final TalonFX m_AlgaeIntakeWristMotor = new TalonFX(HardwareConstants.ALGAE_INTAKE_WRIST_MOTOR_ID);

  /**
   * Algae Intake subsystem constructor.
   */
  public AlgaeIntake() 
  {
    m_AlgaeIntake.getConfigurator().apply(AlgaeIntakeConstants.ALGAE_INTAKE_ROLLER_GAINS);
    m_AlgaeIntake.getConfigurator().apply(AlgaeIntakeConstants.ALGAE_INTAKE_WRIST_CONFIGURATION);
  }

  /**
   * Sets the roller voltage for the Algae Intake roller motor.
   * 
   * @param voltage Takes a paramater of voltage (how fast the motors rotate).
   */
  public void setRollerVoltage(double voltage)
  {
    VoltageOut setVoltage = new VoltageOut(voltage);
    m_AlgaeIntakeRollerMotor.setControl(setVoltage);
  }

  /**
   * Sets the wrist position for the Algae Intake wrist motor.
   * 
   * @param position Takes a paramater of position (where intake arm is located in space).
   */
  public void setWristPosition(double position)
  {
    MotionMagicVoltage setPosition = new MotionMagicVoltage(position, false, AlgaeIntakeConstants.ALGAE_INTAKE_WRIST_FEED_FORWARD, 0, false, false, false);
    m_AlgaeIntakeWristMotor.setControl(setPosition);
  }

  @Override
  public void periodic() 
  {
    // Intentionally Empty.
  }
}
