// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.constants.Hardware;

// Creates a new  Intake subsystem.
public class Intake extends SubsystemBase 
{
  private final TalonFX m_IntakeRollerMotor = new TalonFX(Hardware.INTAKE_ROLLER_MOTOR_ID);
  private final TalonFX m_IntakeWristMotor = new TalonFX(Hardware.INTAKE_WRIST_MOTOR_ID);

  /**
   *  Intake subsystem constructor.
   */
  public Intake() 
  {
    m_IntakeRollerMotor.getConfigurator().apply(IntakeConstants.INTAKE_ROLLER_GAINS);
    m_IntakeWristMotor.getConfigurator().apply(IntakeConstants.INTAKE_WRIST_CONFIGURATION);
  }

  public Command goToPositionThenRoll(double position, double rollerVoltage)
  {
    return new FunctionalCommand(
      // Begin moving the intake to a desired position.
      () -> m_IntakeWristMotor.setControl(new MotionMagicVoltage(position)),
      // Ensure the intake rollers aren't powered while moving the intake.
      () -> m_IntakeRollerMotor.setControl(new VoltageOut(0)),
      // Spin up the intake rollers right before the command ends.
      interrupted ->  m_IntakeRollerMotor.setControl(new VoltageOut(rollerVoltage)),
      // Ends the command once the intake is in the desired position.
      () -> this.getIntakeInPosition(0.1),
      this
  );
  }

  public Command applyWristVoltage(double voltage)
  {
    return this.startEnd(() -> m_IntakeWristMotor.setControl(new VoltageOut(voltage)), () -> m_IntakeWristMotor.setControl(new StaticBrake()));
  }

  public Command applyShooterVoltage(double voltage)
  {
    return this.startEnd(() -> m_IntakeRollerMotor.setControl(new VoltageOut(voltage)), () -> m_IntakeRollerMotor.setControl(new VoltageOut(0)));
  }

  public Command goToIntakePosition()
  {
    return this.runOnce(() -> this.setWristPosition(IntakeConstants.INTAKE_WRIST_GROUND_POSITION));
  }

  public Command goToStowPosition()
  {
    return this.runOnce(() -> this.setWristPosition(IntakeConstants.INTAKE_WRIST_DEFAULT_POSITION));
  }

  private boolean getIntakeInPosition(double tolerance)
  {
    return Math.abs(this.m_IntakeWristMotor.getClosedLoopError().getValueAsDouble()) < tolerance;
  }
  
  /**
   * Sets the roller voltage for the  Intake roller motor.
   * 
   * @param voltage Assigns voltage to the motors (amount of applicable torque).
   */
  private void setRollerVoltage(double voltage)
  {
    VoltageOut setVoltage = new VoltageOut(voltage);
    m_IntakeRollerMotor.setControl(setVoltage);
  }

  /**
   * Sets the wrist position for the  Intake wrist motor.
   * 
   * @param position Assigns wrist position to the motors (where intake arm is located in space).
   */
  private void setWristPosition(double position)
  {
    MotionMagicVoltage setPosition = new MotionMagicVoltage(position);
    m_IntakeWristMotor.setControl(setPosition);
  }

  @Override
  public void periodic() 
  {
    // Intentionally Empty.
  }
}