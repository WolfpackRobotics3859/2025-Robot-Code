// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Hardware;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.MotorManager;

public class Shooter extends SubsystemBase 
{
  private final TalonFX m_ShooterWristMotor;
  private SysIdRoutine m_SysIdRoutine;
  private final MotionMagicVoltage m_WristPositionRequest;

  public Shooter()
  {
    MotorManager.AddMotor("SHOOTER WRIST MOTOR", Hardware.SHOOTER_WRIST_MOTOR);
    m_ShooterWristMotor = MotorManager.GetMotor(Hardware.SHOOTER_WRIST_MOTOR);
    MotorManager.ApplyConfigs(ShooterConstants.WRIST_MOTOR_CONFIG, Hardware.SHOOTER_WRIST_MOTOR);
    m_WristPositionRequest = new MotionMagicVoltage(0);
  }

  public Command StowShooter()
  {
    return this.runOnce(() -> SetWristPositionMotor(ShooterConstants.WRIST_STOW_POSITION));
  }

  public Command MoveToDeployHigh()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION);
  }

  public Command MoveToDeployLow()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION_LOW);
  }

  public Command MoveToIntake()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_CORAL_INTAKE_POSITION);
  }

  public Command MoveToProcess()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_ALGAE_PROCESSOR_DEPLOYMENT_POSITION);
  }

  public Command MoveToAlgaeSweep()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_ALGAE_SWEEPING_POSITION);
  }

  private Command MoveToCommandBuilder(double position)
  {
    return new FunctionalCommand(() -> this.SetWristPositionMotor(position),
                                 () -> {}, 
                                 interrupted -> {},
                                 () -> this.isInPosition(0.025),
                                 this);
  }


  // To-do: Move sysId settings to the constants file
  public SysIdRoutine BuildSysIdRoutine()
  {
    this.m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(0.2).per(Seconds),         // Use default ramp rate (1 V/s)
         Volts.of(0.3), // Reduce dynamic step voltage to 4 to prevent brownout
         null,          // Use default timeout (10 s)
         (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_ShooterWristMotor.setControl(new VoltageOut(volts.in(Volts))),
         null,
         this
      )
   );
   return this.m_SysIdRoutine;
  }

  private void SetWristPositionMotor(double position)
  {
    MotorManager.ApplyControlRequest(m_WristPositionRequest.withPosition(position), Hardware.SHOOTER_WRIST_MOTOR);
  }

  private boolean isInPosition(double tolerance)
  {
    return Math.abs(this.m_ShooterWristMotor.getPosition().getValueAsDouble() - this.m_WristPositionRequest.Position) < tolerance;
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("SHOOTER POSITION", this.m_ShooterWristMotor.getPosition().getValueAsDouble());
    // Intentionally Empty
  }
}
