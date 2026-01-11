// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;
import frc.robot.utilities.MotorManager;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.constants.Hardware;

// Creates a new  Intake subsystem.
public class Intake extends SubsystemBase 
{
  private final TalonFX m_IntakeRollerMotor;
  private final TalonFX m_IntakeWristMotor;

  private final MotionMagicVoltage m_PositionRequest;
  private final VoltageOut m_VoltageRequest;
  private final StaticBrake m_BrakeRequest;

  /**
   *  Intake subsystem constructor.
   */
  public Intake() 
  {
    MotorManager.AddMotor("INTAKE ROLLER MOTOR", Hardware.INTAKE_ROLLER_MOTOR);
    MotorManager.AddMotor("INTAKE WRIST MOTOR", Hardware.INTAKE_WRIST_MOTOR);

    m_IntakeRollerMotor = MotorManager.GetMotor(Hardware.INTAKE_ROLLER_MOTOR);
    m_IntakeWristMotor = MotorManager.GetMotor(Hardware.INTAKE_WRIST_MOTOR);

    MotorManager.ApplyConfigs(IntakeConstants.ROLLER_MOTOR_CONFIG, Hardware.INTAKE_ROLLER_MOTOR);
    MotorManager.ApplyConfigs(IntakeConstants.WRIST_MOTOR_CONFIG, Hardware.INTAKE_WRIST_MOTOR);

    m_PositionRequest = new MotionMagicVoltage(0);
    m_VoltageRequest = new VoltageOut(0);
    m_BrakeRequest = new StaticBrake();
  }

  public Command IntakeRoutine()
  {
    return new SequentialCommandGroup(MoveToPositionWhileRolling(IntakeConstants.WRIST_INTAKING_MID_EXTENSION, IntakeConstants.INTAKING_CLEARING_VOLTAGE),
                                      MoveToPositionWhileRolling(IntakeConstants.WRIST_INTAKING_POSITION, IntakeConstants.INTAKING_VOLTAGE));
  }

  public Command Processing()
  {
    return this.runOnce(() -> this.setWristPosition(IntakeConstants.WRIST_STOW_POSITION).setRollerVoltage(IntakeConstants.PROCESSING_VOLTAGE));
  }

  public Command MoveToPositionWhileRolling(double position, double voltage)
  {
    return new FunctionalCommand(
      // Begin moving the intake to a desired position.
      () -> this.setWristPosition(position).setRollerVoltage(voltage),
      // Ensure the intake rollers aren't powered while moving the intake.
      () -> {},
      // Spin up the intake rollers right before the command ends.
      interrupted -> {},
      // Ends the command once the intake is in the desired position.
      MotorManager.InPosition(Hardware.INTAKE_WRIST_MOTOR, 0.05),
      this
    );
  }

  public Command StowIntake()
  {
    return this.runOnce(() -> this.BrakeRoller().setWristPosition(IntakeConstants.WRIST_STOW_POSITION));
  }
  
  /**
   * Sets the roller voltage for the  Intake roller motor.
   * 
   * @param voltage Assigns voltage to the motors (amount of applicable torque).
   */
  private Intake setRollerVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_VoltageRequest.withOutput(voltage), Hardware.INTAKE_ROLLER_MOTOR);
    return this;
  }

  /**
   * Sets the wrist position for the  Intake wrist motor.
   * 
   * @param position Assigns wrist position to the motors (where intake arm is located in space).
   */
  private Intake setWristPosition(double position)
  {
    MotorManager.ApplyControlRequest(m_PositionRequest.withPosition(position), Hardware.INTAKE_WRIST_MOTOR);
    return this;
  }

  private Intake BrakeRoller()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.INTAKE_ROLLER_MOTOR);
    return this;
  }

  private Intake BrakeWrist()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.INTAKE_WRIST_MOTOR);
    return this;
  }

  @Override
  public void periodic() 
  {
    // Intentionally Empty.
  }
}